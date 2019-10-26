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

		imu_hw_ = robot_hw_->get<hardware_interface::ImuSensorInterface>();
		if (!imu_hw_)
		{
			ROS_ERROR("This controller requires a hardware interface of type ImuSensorInterface");
			return false;
		}

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

		std::string imu_name;
		if(!node_.getParam("imu_name",imu_name))
        {
            ROS_ERROR("No 'imu' in controller. (namespace: %s)",
                    node_.getNamespace().c_str());
            return false;
        }

     	try
		{
			imu_handle_ = imu_hw_->getHandle(imu_name);
		}
		catch (const hardware_interface::HardwareInterfaceException &e)
		{
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
			return false;
		}

		// Command Topic
        sub_command_ = node_.subscribe("command", 1,
                    &PlatformComputedTorqueController::commandCB, this);
        
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

        KDL::Vector g;
        node_.param("gravity/x",g[0],0.0);
        node_.param("gravity/y",g[1],0.0);
        node_.param("gravity/z",g[2],-9.8);
        
        if((idsolver_=new::KDL::ChainIdSolver_RNE(chain_,g))==NULL)
        {
            ROS_ERROR("Failed to create ChainIDSolver_RNE.");
            return false;
        }

		q_.resize(nJoints_);
		dq_.resize(nJoints_);
		v_.resize(nJoints_);
		qr_.resize(nJoints_);
		dqr_.resize(nJoints_);
		ddqr_.resize(nJoints_);
		torque_.resize(nJoints_);
        fext_.resize(chain_.getNrOfSegments());

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
		
		return true;
	}

    void PlatformComputedTorqueController::starting(const ros::Time &time)
    {
		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i)=joints_[i].getPosition();
			dq_(i)=joints_[i].getVelocity();
		}
		qr_=q_;
		dqr_=dq_;
		SetToZero(ddqr_);

       	for (unsigned int i = 0; i < 3; i++)
		{
			quatp_(i) = imu_handle_.getOrientation()[i];
		}
        for(unsigned int i=0;i < 2;i++)
        {
            dqp_(i) = imu_handle_.getAngularVelocity()[i];
            ddqp_(i) = imu_handle_.getLinearAcceleration()[i]; //TODO get angular acceleration
        }
		KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3)).GetRPY(
		qp_(2),qp_(0),qp_(1));
		
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
		for (unsigned int i = 0; i < 3; i++)
		{
			quatp_(i) = imu_handle_.getOrientation()[i];
		}
        for(unsigned int i=0;i < 2;i++)
        {
            dqp_(i) = imu_handle_.getAngularVelocity()[i];
            ddqp_(i) = imu_handle_.getLinearAcceleration()[i]; //TODO get angular acceleration
        }
		KDL::Rotation::Quaternion(quatp_(0),quatp_(1),quatp_(2),quatp_(3)).GetRPY(
			qp_(2),qp_(0),qp_(1));

		for(unsigned int i=0;i < nJoints_;i++)
		{
			q_(i)=joints_[i].getPosition();
			dq_(i)=joints_[i].getVelocity();
		}
		for(unsigned int i=0;i < fext_.size();i++) fext_[i].Zero();
		
		v_.data=ddqr_.data+Kp_*(qr_.data-q_.data)+Kd_*(dqr_.data-dq_.data);
		if(idsolver_->CartToJnt(q_,dq_,v_,fext_,torque_) < 0)
		        ROS_ERROR("KDL inverse dynamics solver failed.");
		
		for(unsigned int i=0;i < nJoints_;i++)
		        joints_[i].setCommand(torque_(i));
	}
	
	void PlatformComputedTorqueController::commandCB(const trajectory_msgs::
	        JointTrajectoryPoint::ConstPtr &referencePoint)
	{
		for(unsigned int i=0;i < nJoints_;i++)
		{
			qr_(i)=referencePoint->positions[i];
			dqr_(i)=referencePoint->velocities[i];
			ddqr_(i)=referencePoint->accelerations[i];
		}                
    }

}

PLUGINLIB_EXPORT_CLASS(effort_controllers::PlatformComputedTorqueController,
                        controller_interface::ControllerBase)
//eof