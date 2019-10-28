#ifndef PLATFORM_TORQUE_CONTROLLER_PLATFORM_TORQUE_CONTROLLER
#define PLATFORM_TORQUE_CONTROLLER_PLATFORM_TORQUE_CONTROLLER

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

namespace effort_controllers
{
    class PlatformComputedTorqueController: public controller_interface::
        MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::ImuSensorInterface>
    {
        ros::NodeHandle node_;

        hardware_interface::RobotHW *robot_hw_;
        hardware_interface::EffortJointInterface *effort_hw_;
        std::vector<hardware_interface::JointHandle> joints_;
        
        int nJoints_;
        int nJointsVirt_;
        
        // IMU
        hardware_interface::ImuSensorHandle imu_handle_;
        hardware_interface::ImuSensorInterface *imu_hw_;

        ros::Subscriber sub_command_;
        ros::Subscriber sub_imu_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        KDL::ChainIdSolver_RNE *idsolver_;

        KDL::JntArray q_;
        KDL::JntArray dq_;
        KDL::JntArray v_;
        KDL::JntArray qr_;
		KDL::JntArray dqr_;
		KDL::JntArray ddqr_;

        //Platform angles: qp_(0)=pitch qp_(1)=roll
        // remember to get quaternion from orientation and convert to roll and pitch
        KDL::JntArray quatp_;
        KDL::JntArray qp_;
        KDL::JntArray dqp_;
        KDL::JntArray ddqp_;
			
		KDL::JntArray torque_;
        KDL::Wrenches fext_;
			
		Eigen::MatrixXd Kp_;
		Eigen::MatrixXd Kd_;

        Eigen::MatrixXd KpVirt_;
        Eigen::MatrixXd KdVirt_;

        void commandCB(const trajectory_msgs::JointTrajectoryPoint::
        ConstPtr &reference);
        void imuCB(const sensor_msgs::Imu::ConstPtr &imu_data);

        public:
        PlatformComputedTorqueController(void);
        ~PlatformComputedTorqueController(void);
        
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &n);
        void starting(const ros::Time &time);
        void update(const ros::Time &time, const ros::Duration &duration);

    };
}

#endif