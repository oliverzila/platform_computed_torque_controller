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
		std::cout<<"Size joints_: "<<joints_.size()<<std::endl;
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

		std::string chainRoot;
		if(!node_.getParam("chain/root",chainRoot))
		{
			ROS_ERROR("Could not find 'chain_root' parameter.");
			return false;
		}
		
		std::string chainTip;
		if(!node_.getParam("chain/tip",chainTip))
		{
			ROS_ERROR("Could not find 'chain/tip' parameter.");
			return false;
		}
		
		if (!tree_.getChain(chainRoot,chainTip,chain_)) 
		{
			ROS_ERROR("Failed to get chain from KDL tree.");
			return false;
		}

		// chain_platform_ = KDL::Chain();
		// KDL::Joint joint1_(KDL::Joint::None); // inertial ref and platform
		// KDL::Frame frame1_ = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));
		// chain_platform_.addSegment(KDL::Segment(joint1_,frame1_));

		// KDL::Joint joint_pitch_(KDL::Joint::RotY); // pitch joint
		// KDL::Frame frame_pitch_ = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));
		// chain_platform_.addSegment(KDL::Segment(joint_pitch_,frame_pitch_));
		
		// KDL::Joint joint_roll_(KDL::Joint::RotX); // roll joint
		// KDL::Frame frame_roll_ = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));
		// chain_platform_.addSegment(KDL::Segment(joint_roll_,frame_roll_));

		std::string platformRoot;
		if(!node_.getParam("platform_chain/root",platformRoot))
		{
			ROS_ERROR("Could not find 'platform_root' parameter.");
			return false;
		}
		
		std::string platformTip;
		if(!node_.getParam("platform_chain/tip",platformTip))
		{
			ROS_ERROR("Could not find 'chain/tip' parameter.");
			return false;
		}
		
		if (!tree_.getChain(platformRoot,chainTip,chain_platform_)) 
		{
			ROS_ERROR("Failed to get chain from KDL tree.");
			return false;
		}

		//chain_platform_.addChain(chain_);

        KDL::Vector g;
        node_.param("gravity/x",g[0],0.0);
        node_.param("gravity/y",g[1],0.0);
        node_.param("gravity/z",g[2],-9.8);
        
        if((idsolver_=new::KDL::ChainIdSolver_RNE(chain_platform_,g))==NULL)
        {
            ROS_ERROR("Failed to create ChainIDSolver_RNE.");
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
        fext_.resize(chain_platform_.getNrOfSegments());

		quatp_.resize(4);
        qp_.resize(DOF);
        dqp_.resize(DOF);
        ddqp_.resize(DOF);
		
		Kp_.resize(nJoints_,nJoints_);
		Kd_.resize(nJoints_,nJoints_);        

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
		
		KpVirt_ = Eigen::MatrixXd::Zero(nJointsVirt_, nJointsVirt_);
		KpVirt_.bottomRightCorner(nJoints_,nJoints_) = Kp_;
		KdVirt_ = Eigen::MatrixXd::Zero(nJointsVirt_, nJointsVirt_);
		KpVirt_.bottomRightCorner(nJoints_,nJoints_) = Kd_;

		return true;
	}

    void PlatformComputedTorqueController::starting(const ros::Time &time)
    {
		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	quatp_(i) = imu_handle_.getOrientation()[i];
		// }
        // for(unsigned int i=0;i < 2;i++)
        // {
        //     dqp_(i) = imu_handle_.getAngularVelocity()[i];
        //     ddqp_(i) = imu_handle_.getLinearAcceleration()[i];
        // }
		// KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3)).GetRPY(
		// qp_(2),qp_(0),qp_(1));

		// robot joints initial condition
		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i+DOF)=joints_[i].getPosition();
			std::cout<<"Posicao inicial junta "<<i<<": "<<q_(i+DOF)<<std::endl;
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
		// for (unsigned int i = 0; i < 3; i++)
		// {
		// 	quatp_(i) = imu_handle_.getOrientation()[i];
		// }
        // for(unsigned int i=0;i < 2;i++)
        // {
        //     dqp_(i) = imu_handle_.getAngularVelocity()[i];
        //     ddqp_(i) = imu_handle_.getLinearAcceleration()[i];
        // }
		// KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3)).GetRPY(
		// 	qp_(2),qp_(0),qp_(1));

		// Get robot joints state
		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i+DOF)=joints_[i].getPosition();
			dq_(i+DOF)=joints_[i].getVelocity();
		}
		// Get platform orientation
		for(unsigned int i=0;i < DOF;i++)
		{
			q_(i)=qp_(i);
			dq_(i)=dqp_(i);

			qr_(i)=q_(i);
			dqr_(i)=dq_(i);
		}
		for(unsigned int i=0;i < fext_.size();i++) fext_[i].Zero();

		v_.data=ddqr_.data+KpVirt_*(qr_.data-q_.data)+KdVirt_*(dqr_.data-dq_.data);
		std::cout<<"Segmentos: "<<chain_platform_.getNrOfSegments()<<std::endl;
		std::cout<<"Juntas: "<<chain_platform_.getNrOfJoints()<<std::endl;
		if(idsolver_->CartToJnt(q_,dq_,v_,fext_,torque_) < 0)
		        ROS_ERROR("KDL inverse dynamics solver failed.");
		for(int i=0;i<nJointsVirt_;i++)
			std::cout<<"qr: "<<qr_(i)<<"q"<<i<<": "<<q_(i)<<" dq"<<i<<": "<<dq_(i)<<" v_"<<i<<": "<<v_(i)<<" torque"<<i<<": "<<torque_(i)<<std::endl;
		for(unsigned int i=0;i < nJoints_;i++)
		        joints_[i].setCommand(torque_(i+DOF));
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
		quatp_(0) = 0.0; //imu_data->orientation.x;
		quatp_(1) = 0.0; //imu_data->orientation.y;
		quatp_(2) = 0.0; //imu_data->orientation.z;
		quatp_(3) = 1.0; //imu_data->orientation.w;
		      
        dqp_(0) = 0.0; //imu_data->angular_velocity.x;
		dqp_(1) = 0.0; //imu_data->angular_velocity.y;
		dqp_(2) = 0.0; //imu_data->angular_velocity.z;

		// TODO convert to angular acceleration
        ddqp_(0) = 0.0; //imu_data->linear_acceleration.x;
		ddqp_(1) = 0.0; //imu_data->linear_acceleration.y;
		ddqp_(2) = 0.0; //imu_data->linear_acceleration.z;
		KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3)).GetRPY(
			qp_(2),qp_(0),qp_(1));

	}

}

PLUGINLIB_EXPORT_CLASS(effort_controllers::PlatformComputedTorqueController,
                        controller_interface::ControllerBase)
//eof