#ifndef PLATFORM_TORQUE_CONTROLLER_PLATFORM_TORQUE_CONTROLLER
#define PLATFORM_TORQUE_CONTROLLER_PLATFORM_TORQUE_CONTROLLER

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

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
        Controller<hardware_interface::EffortJointInterface>
    {
        ros::NodeHandle node_;

        hardware_interface::EffortJointInterface *hw_;
        //TODO check if should use imu interface instead
        hardware_interface::ImuSensorHandle imu_;
        std::vector<hardware_interface::JointHandle> joints_;
        int nJoints_;

        ros::Subscriber sub_command_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        KDL::ChainIdSolver_RNE *idsolver_;

        KDL::JntArray q_;
        KDL::JntArray dq_;
        KDL::JntArray v_;
        KDL::JntArray qr_;
		KDL::JntArray dqr_;
		KDL::JntArray ddqr_;
			
		KDL::JntArray torque_;
        KDL::Wrenches fext_;
			
		Eigen::MatrixXd Kp_;
		Eigen::MatrixXd Kd_;

        void commandCB(const trajectory_msgs::JointTrajectoryPoint::
        ConstPtr &reference);

        public:
        PlatformComputedTorqueController(void);
        ~PlatformComputedTorqueController(void);

        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);
        void starting(const ros::Time &time);
        void update(const ros::Time &time, const ros::Duration &duration);

    };
}

#endif