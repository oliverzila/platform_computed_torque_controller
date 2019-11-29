/******************************************************************************
		    ROS platform_computed_torque_controller Package
                    Platform Computed Torque  Controller
          		Copyright (C) 2019 Carlos Eduardo Pedroso de Oliveira

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

/*
	Based on the ROS computed_torque_controller Package
	Author: Walter Fetter Lages
*/

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
        KDL::Chain imuChain_;
        KDL::ChainIdSolver_RNE *idsolver_;

        KDL::JntArray q_;
        KDL::JntArray dq_;
        KDL::JntArray v_;
        KDL::JntArray qr_;
		KDL::JntArray dqr_;
		KDL::JntArray ddqr_;
        KDL::JntArray qe_int_;

        //Platform angles: qp_(0)=roll qp_(1)=pitch
        KDL::JntArray quatp_;
        KDL::JntArray qp_;
        KDL::JntArray dqp_;
        KDL::JntArray ddqp_;
        // Notation r_a_b = ^{b}R_{a}
        KDL::Rotation r_imu_t_;
        Eigen::MatrixXd mr_imu_t_;
        KDL::Rotation r_p_enu_;
        KDL::JntArray w_imu_imu; // sensor angular rate measure
        KDL::JntArray a_imu_imu; // sensor linear acceleration measure
        

        KDL::Rotation r_enu_0_;
        Eigen::MatrixXd mr_enu_0_;
		
        Eigen::VectorXd gravity_v_;

		KDL::JntArray torque_;
        KDL::Wrenches fext_;
			
		Eigen::MatrixXd Kp_;
		Eigen::MatrixXd Kd_;
        Eigen::MatrixXd Ki_;

        Eigen::MatrixXd KpVirt_;
        Eigen::MatrixXd KdVirt_;
        Eigen::MatrixXd KiVirt_; 

        double now_time;
        double last_time;

        void commandCB(const trajectory_msgs::JointTrajectoryPoint::
        ConstPtr &reference);
        void imuCB(const sensor_msgs::Imu::ConstPtr &imu_data);

        public:
        PlatformComputedTorqueController(void);
        ~PlatformComputedTorqueController(void);
        
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &n);
        void starting(const ros::Time &time);
        void update(const ros::Time &time, const ros::Duration &duration);
        void wJacobian(KDL::JntArray qp, Eigen::MatrixXd &wJac);
        void mRotation2Matrix(KDL::Rotation rot, Eigen::MatrixXd &matrix);
    };
}

#endif