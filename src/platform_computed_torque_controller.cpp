#include <sys/mman.h>

#include <platform_computed_torque_controller/platform_computed_torque_controller.h>
#include <pluginlib/class_list_macros.h>

#define DOF 2

namespace effort_controllers
{
    PlatformComputedTorqueController::PlatformComputedTorqueController(void):
		controller_interface::
        MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::ImuSensorInterface> (true),
		q_(0),dq_(0),v_(0),qr_(0),dqr_(0),ddqr_(0),torque_(0),fext_(0), qp_(0),dqp_(0),ddqp_(0)
    {  
    }

    PlatformComputedTorqueController::~PlatformComputedTorqueController(void)
    {
        sub_command_.shutdown();
		sub_imu_.shutdown();
    }

    bool PlatformComputedTorqueController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &n)
    {

		node_ = n;
		robot_hw_ = robot_hw;

		effort_hw_ = robot_hw_->get<hardware_interface::EffortJointInterface>();

		if (!effort_hw_)
		{
			ROS_ERROR("This controller requires a hardware interface of type hardware_interface::EffortJointInterface");
			return false;
		}

		// imu_hw_ = robot_hw_->get<hardware_interface::ImuSensorInterface>();
		// if (!imu_hw_)
		// {
		// 	ROS_ERROR("This controller requires a hardware interface of type ImuSensorInterface");
		// 	return false;
		// }

        std::vector<std::string> joint_names;
        if(!node_.getParam("joints",joint_names))
        {
            ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                    node_.getNamespace().c_str());
            return false;
        }

        nJoints_=joint_names.size();

     	for (int i = 0; i < nJoints_; i++)
	    {
		    try
	    	{
	    		joints_.push_back(effort_hw_->getHandle(joint_names[i]));
	    	}
	    	catch (const hardware_interface::HardwareInterfaceException &e)
	    	{
	    		ROS_ERROR_STREAM("Exception thrown: " << e.what());
	    		return false;
	    	}
	    }

		// std::string imu_name;
		// if(!node_.getParam("imu_name",imu_name))
        // {
        //     ROS_ERROR("No 'imu' in controller. (namespace: %s)",
        //             node_.getNamespace().c_str());
        //     return false;
        // }

     	// try
		// {
		// 	imu_handle_ = imu_hw_->getHandle(imu_name);
		// }
		// catch (const hardware_interface::HardwareInterfaceException &e)
		// {
		// 	ROS_ERROR_STREAM("Exception thrown: " << e.what());
		// 	return false;
		// }

		// Command Topic
        sub_command_ = node_.subscribe("command", 1,
                    &PlatformComputedTorqueController::commandCB, this);

		// IMU Topic
        sub_imu_ = node_.subscribe("imu/data", 1,
                    &PlatformComputedTorqueController::imuCB, this);
        
        std::string robot_desc_string;
        if(!node_.getParam("/robot_description", robot_desc_string))
        {
            ROS_ERROR("Could not find /'robot_description'.");
            return false;
        }

        if(!kdl_parser::treeFromString(robot_desc_string, tree_))
        {
            ROS_ERROR("KDL tree failed to construct");
            return false;
        }
		
		//ROBOT + PLATFORM CHAIN
		std::string chainTip;
		if(!node_.getParam("chain/tip",chainTip))
		{
			ROS_ERROR("Could not find 'chain/tip' parameter.");
			return false;
		}
		std::string platformRoot;
		if(!node_.getParam("platform_chain/root",platformRoot))
		{
			ROS_ERROR("Could not find 'platform_root' parameter.");
			return false;
		}
		if (!tree_.getChain(platformRoot,chainTip,chain_)) 
		{
			ROS_ERROR("Could not find 'chain/tip' parameter.");
			return false;
		}
		//END ROBOT + PLATFORM CHAIN

		// IMU CHAIN
		std::string imuLink;
		if(!node_.getParam("imu_chain/tip",imuLink))
		{
			ROS_ERROR("Could not find 'imu_chain/tip' parameter.");
			return false;
		}
		if (!tree_.getChain(platformRoot,imuLink,imuChain_)) 
		{
			ROS_ERROR("Could not find 'chain/tip' parameter.");
			return false;
		}
		// END IMU Chain

        KDL::Vector g;
        node_.param("gravity/x",g[0],0.0);
        node_.param("gravity/y",g[1],0.0);
        node_.param("gravity/z",g[2],-9.8);
		mVectorEigen(g,gravity_v_);
        
        if((idsolver_=new::KDL::ChainIdSolver_RNE(chain_,g))==NULL)
        {
            ROS_ERROR("Failed to create ChainIDSolver_RNE.");
            return false;
        }
		if((jdsolver_=new::KDL::ChainJntToJacDotSolver(imuChain_))==NULL)
        {
            ROS_ERROR("Failed to create ChainJntToJacDotSolver.");
            return false;
        }
		if((jsolver_=new::KDL::ChainJntToJacSolver(imuChain_))==NULL)
        {
            ROS_ERROR("Failed to create ChainJntToJacDotSolver.");
            return false;
        }
		
		nJointsVirt_ = nJoints_+DOF;
		q_.resize(nJointsVirt_);
		dq_.resize(nJointsVirt_);
		v_.resize(nJointsVirt_);
		qr_.resize(nJointsVirt_);
		dqr_.resize(nJointsVirt_);
		ddqr_.resize(nJointsVirt_);
		torque_.resize(nJointsVirt_);
		qe_int_.resize(nJointsVirt_);
        fext_.resize(chain_.getNrOfSegments());

		quatp_.resize(4);
        qp_.resize(3);
        dqp_.resize(3);
        ddqp_.resize(3);
		
		w_imu_imu.resize(3);
		a_imu_imu.resize(3);

		// TODO parse from urdf instead of getting from param server
		// Imu -> Platform top
		KDL::JntArray imu_joint_rpy_;
		imu_joint_rpy_.resize(3);
		if(!node_.getParam("imu_joint/roll",imu_joint_rpy_(0)))
		{
			ROS_WARN("No roll defined for imu, assuming zero");
			imu_joint_rpy_(0)=0.0;
		}
		if(!node_.getParam("imu_joint/pitch",imu_joint_rpy_(1)))
		{
			ROS_WARN("No pitch defined for imu, assuming zero");
			imu_joint_rpy_(1)=0.0;
		}
		if(!node_.getParam("imu_joint/yaw",imu_joint_rpy_(2)))
		{
			ROS_WARN("No yaw defined for imu, assuming zero");
			imu_joint_rpy_(2)=0.0;
		}
		r_imu_t_=KDL::Rotation::RPY(imu_joint_rpy_(0),
		imu_joint_rpy_(1),imu_joint_rpy_(2));
		mRotation2Matrix(r_imu_t_, mr_imu_t_);
		// Enu -> World
		if(!node_.getParam("enu_world/roll",imu_joint_rpy_(0)))
		{
			ROS_WARN("No enu_world/roll defined, assuming zero");
			imu_joint_rpy_(0)=0.0;
		}
		if(!node_.getParam("enu_world/pitch",imu_joint_rpy_(1)))
		{
			ROS_WARN("No pitch defined for imu, assuming zero");
			imu_joint_rpy_(1)=0.0;
		}
		if(!node_.getParam("enu_world/yaw",imu_joint_rpy_(2)))
		{
			ROS_WARN("No yaw defined for imu, assuming zero");
			imu_joint_rpy_(2)=0.0;
		}
		r_enu_0_=KDL::Rotation::RPY(imu_joint_rpy_(0),
		imu_joint_rpy_(1),imu_joint_rpy_(2));
		mRotation2Matrix(r_enu_0_,mr_enu_0_);
		// Platform -> Enu
		if(!node_.getParam("platform_enu/roll",imu_joint_rpy_(0)))
		{
			ROS_WARN("No platform_enu/roll defined, assuming zero");
			imu_joint_rpy_(0)=0.0;
		}
		if(!node_.getParam("platform_enu/pitch",imu_joint_rpy_(1)))
		{
			ROS_WARN("No pitch defined for imu, assuming zero");
			imu_joint_rpy_(1)=0.0;
		}
		if(!node_.getParam("platform_enu/yaw",imu_joint_rpy_(2)))
		{
			ROS_WARN("No yaw defined for imu, assuming zero");
			imu_joint_rpy_(2)=0.0;
		}
		r_p_enu_=KDL::Rotation::RPY(imu_joint_rpy_(0),
		imu_joint_rpy_(1),imu_joint_rpy_(2));
		// remaining parameters -- TO BE REMOVED --
		// if(!node_.getParam("platform_imu_top/x",pimu_t(0)))
		// {
		// 	ROS_WARN("No platform_imu_top/x defined for imu, assuming zero");
		// 	pimu_t(0)=0.0;
		// }
		// if(!node_.getParam("platform_imu_top/y",pimu_t(1)))
		// {
		// 	ROS_WARN("No platform_imu_top/y defined for imu, assuming zero");
		// 	pimu_t(1)=0.0;
		// }
		// if(!node_.getParam("platform_imu_top/z",pimu_t(2)))
		// {
		// 	ROS_WARN("No platform_imu_top/x defined for imu, assuming zero");
		// 	pimu_t(0)=0.0;
		// }

		// if(!node_.getParam("platform_top_base/x",pt_p(0)))
		// {
		// 	ROS_WARN("No platform_top_base/x defined for imu, assuming zero");
		// 	pt_p(0)=0.0;
		// }
		// if(!node_.getParam("platform_top_base/y",pt_p(1)))
		// {
		// 	ROS_WARN("No platform_top_base/y defined for imu, assuming zero");
		// 	pt_p(1)=0.0;
		// }
		// if(!node_.getParam("platform_top_base/z",pt_p(2)))
		// {
		// 	ROS_WARN("No platform_top_base/x defined for imu, assuming zero");
		// 	pt_p(0)=0.0;
		// }
		
		Kp_.resize(nJoints_,nJoints_);
		Kd_.resize(nJoints_,nJoints_);
		Ki_.resize(nJoints_,nJoints_);

		std::vector<double> KpVec;
		if(!node_.getParam("Kp",KpVec))
		{
			ROS_ERROR("No 'Kp' in controller %s.",node_.getNamespace().c_str());
			return false;
		}
		Kp_=Eigen::Map<Eigen::MatrixXd>(KpVec.data(),nJoints_,nJoints_).transpose();
		
		std::vector<double> KdVec;
		if(!node_.getParam("Kd",KdVec))
		{
			ROS_ERROR("No 'Kd' in controller %s.",node_.getNamespace().c_str());
			return false;
		}
		Kd_=Eigen::Map<Eigen::MatrixXd>(KdVec.data(),nJoints_,nJoints_).transpose();

		std::vector<double> KiVec;
		if(!node_.getParam("Ki",KiVec))
		{
			ROS_ERROR("No 'Ki' in controller %s.",node_.getNamespace().c_str());
			return false;
		}
		Ki_=Eigen::Map<Eigen::MatrixXd>(KiVec.data(),nJoints_,nJoints_).transpose();
		
		KpVirt_ = Eigen::MatrixXd::Zero(nJointsVirt_, nJointsVirt_);
		KpVirt_.bottomRightCorner(nJoints_,nJoints_) = Kp_;
		KdVirt_ = Eigen::MatrixXd::Zero(nJointsVirt_, nJointsVirt_);
		KdVirt_.bottomRightCorner(nJoints_,nJoints_) = Kd_;
		KiVirt_ = Eigen::MatrixXd::Zero(nJointsVirt_, nJointsVirt_);
		KiVirt_.bottomRightCorner(nJoints_,nJoints_) = Ki_;

		return true;
	}

    void PlatformComputedTorqueController::starting(const ros::Time &time)
    {
		// robot joints initial condition
		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i+DOF)=joints_[i].getPosition();
			dq_(i+DOF)=joints_[i].getVelocity();
			qr_(i+DOF)=q_(i+DOF);
			dqr_(i+DOF)=dq_(i+DOF);
		}

		// platform initial conditions
		for(unsigned int i=0;i < DOF;i++)
		{
			q_(i)= 0.0; //qp_(i);
			dq_(i)= 0.0; //dqp_(i);
			qr_(i)=q_(i);
			dqr_(i)=dq_(i);
		}
		SetToZero(ddqr_);

		last_time = ros::Time::now().toSec();
		
		struct sched_param param;
		if(!node_.getParam("priority",param.sched_priority))
		{
			ROS_WARN("No 'priority' configured for controller %s. Using highest possible priority.",node_.getNamespace().c_str());
			param.sched_priority=sched_get_priority_max(SCHED_FIFO);	
		}
		if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
		{
			ROS_WARN("Failed to set real-time scheduler.");
			return;
		}
		if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
			ROS_WARN("Failed to lock memory.");    
    }    

    void PlatformComputedTorqueController::update(const ros::Time &time,
            const ros::Duration &duration)
    {
		// Get robot joints state
		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i+DOF)=joints_[i].getPosition();
			dq_(i+DOF)=joints_[i].getVelocity();
		}

		// BEGIN PLATFORM STATE
		// oreintation calculation
		KDL::Rotation r_imu_enu = KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3));
		
		Eigen::MatrixXd mr_imu_enu;
		mRotation2Matrix(r_imu_enu, mr_imu_enu);
		KDL::Rotation r_t_p = r_p_enu_*r_imu_enu*r_imu_t_.Inverse();

        Eigen::MatrixXd mr_t_p;
		mRotation2Matrix(r_t_p,mr_t_p);
		// platform orientation as joint angles
		r_t_p.GetRPY(qp_(0),qp_(1),qp_(2));

		// angular velocity calculation
		Eigen::MatrixXd w_jac_;
		wJacobian(qp_, w_jac_);

		dqp_.data = w_jac_.transpose()*mr_t_p*mr_imu_t_*w_imu_imu.data;

		// angular acceleration calculation
		qpvel_.q.data=qp_.data.head(2);
		qpvel_.qdot=dqp_;
		if (jdsolver_->JntToJacDot(qpvel_, djac_, -1) != 0)
		{
			ROS_ERROR("KDL Jacobian dot solver failed");
		}
		if (jsolver_->JntToJac(qpvel_.q,jac_) != 0)
		{
			ROS_ERROR("KDL Jacobian solver failed");
		}
        Eigen::MatrixXd v_jac_;
		vJacobian(qp_, v_jac_);
		// vJacobianDot(qp_, dqp_, vJacDot_);
		pseudoInv(v_jac_, vJacInv_);
		//ddqp_.data = vJacInv_*(mr_t_p*mr_imu_t_*(a_imu_imu.data+mr_enu_0_*mr_imu_enu*gravity_v_)-djac_.data*dqp_.data);
		for(unsigned int i=0;i < DOF;i++)
		{
			q_(i)= qp_(i);
			dq_(i)= dqp_(i);
			
			qr_(i)=q_(i);
			dqr_(i)=dq_(i);
		}
		//END PLATFORM STATE

		for(unsigned int i=0;i < fext_.size();i++) fext_[i].Zero();

		now_time = ros::Time::now().toSec();

		qe_int_.data+=(qr_.data-q_.data)*(now_time-last_time);
		v_.data=ddqr_.data+KpVirt_*(qr_.data-q_.data)+KdVirt_*(dqr_.data-dq_.data)+KiVirt_*qe_int_.data;

		for (int i=0;i<DOF;i++)	v_(i)=0.0;//ddqp_(i); // platform joint accelerations

		if(idsolver_->CartToJnt(q_,dq_,v_,fext_,torque_) < 0)
		        ROS_ERROR("KDL inverse dynamics solver failed.");
		for(unsigned int i=0;i < nJoints_;i++)
		        joints_[i].setCommand(torque_(i+DOF));
		
		last_time=now_time;
		/* ----TESTES IMU---- */
		std::cout<<"|-----------------------------------------|"<<std::endl;
		std::cout<<"Roll: "<<qp_(0)<<" Pitch: "<<qp_(1)<<" Yaw: "<<qp_(2)<<std::endl;
		std::cout<<"Wx: "<<dqp_(0)<<" Wy: "<<dqp_(1)<<" Wz: "<<dqp_(2)<<std::endl;
		std::cout<<"QVEL Q: \n"<<qpvel_.q.data<<std::endl;
		std::cout<<"QVEL dQ: \n"<<qpvel_.qdot.data<<std::endl;
		std::cout<<"# of joints imuChain_: \n"<<imuChain_.getNrOfJoints()<<std::endl;

	}


	
	void PlatformComputedTorqueController::commandCB(const trajectory_msgs::
	        JointTrajectoryPoint::ConstPtr &referencePoint)
	{
		for(unsigned int i=0;i < nJoints_;i++)
		{
			qr_(i+DOF)=referencePoint->positions[i];
			dqr_(i+DOF)=referencePoint->velocities[i];
			ddqr_(i+DOF)=referencePoint->accelerations[i];
		}                
    }

	void PlatformComputedTorqueController::imuCB(const sensor_msgs::Imu::ConstPtr &imu_data)
	{
		// sensor reading
		quatp_(0) = imu_data->orientation.x;
		quatp_(1) = imu_data->orientation.y;
		quatp_(2) = imu_data->orientation.z;
		quatp_(3) = imu_data->orientation.w;
		      
        w_imu_imu(0) = imu_data->angular_velocity.x;
		w_imu_imu(1) = imu_data->angular_velocity.y;
		w_imu_imu(2) = imu_data->angular_velocity.z;

		a_imu_imu(0) = imu_data->linear_acceleration.x;
		a_imu_imu(1) = imu_data->linear_acceleration.y;
		a_imu_imu(2) = imu_data->linear_acceleration.z;

		
	}

	void PlatformComputedTorqueController::wJacobian(KDL::JntArray qp, Eigen::MatrixXd &wJac)
	{ //TODO update
		wJac = (Eigen::MatrixXd(3,2) << 
		0.0, std::sin(qp(0)), 
		0.0, -std::cos(qp(0)),
		1.0, 0.0).finished();
	}
	void PlatformComputedTorqueController::vJacobian(KDL::JntArray qp, Eigen::MatrixXd &vJac)
	{ //TODO update
		vJac = (Eigen::MatrixXd(3,2) << 
		1.0, 0.0, 
		0.0, std::cos(qp(0)),
		0.0, std::sin(qp(0))).finished();
	}
	void PlatformComputedTorqueController::vJacobianDot(KDL::JntArray qp, KDL::JntArray dqp, Eigen::MatrixXd &vdJac)
	{ //TODO updade
		vdJac = (Eigen::MatrixXd(3,2) << 
		1.0, 0.0, 
		0.0, std::cos(qp(0)),
		0.0, std::sin(qp(0))).finished();
	}
	void PlatformComputedTorqueController::pseudoInv(Eigen::MatrixXd &Jac, Eigen::MatrixXd &invJac)
	{
		//invJac = ((Jac.transpose()*Jac).inverse())*Jac.transpose();
	}
	

	void PlatformComputedTorqueController::mRotation2Matrix(KDL::Rotation rot, Eigen::MatrixXd &matrix)
	{
		matrix = (Eigen::MatrixXd(3,3) << 
		rot.data[0], rot.data[1], rot.data[2], 
		rot.data[3], rot.data[4], rot.data[5], 
		rot.data[6], rot.data[7], rot.data[8]).finished();
	}

	void PlatformComputedTorqueController::mVectorEigen(KDL::Vector vec, Eigen::VectorXd &eigenV)
	{
		eigenV = (Eigen::VectorXd(3) << vec.data[0], vec.data[1], vec.data[2]).finished();
	}


}

PLUGINLIB_EXPORT_CLASS(effort_controllers::PlatformComputedTorqueController,
                        controller_interface::ControllerBase)
//eof