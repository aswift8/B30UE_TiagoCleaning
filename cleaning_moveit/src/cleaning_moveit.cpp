// ROS headers
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <iostream>

std::vector<double> jointAngles;

void positionCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
	// Copy joint positions to jointAngles
	jointAngles = msg->actual.positions;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "cleaning_moveit");

	ros::NodeHandle n;
	ros::Subscriber subJointAngles = n.subscribe("/arm_controller/state", 2, positionCallback);

	ros::Publisher pubJacobian = n.advertise<std_msgs::Float32MultiArray>("/cleaning/jacobian", 1);
	ros::Publisher pubFKPos = n.advertise<geometry_msgs::Vector3>("/cleaning/forward_kinematics/position", 1);
	ros::Publisher pubFKOri = n.advertise<std_msgs::Float32MultiArray>("/cleaning/forward_kinematics/orientation", 1);

	// Select group of joints
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	moveit::planning_interface::MoveGroup group("arm"); // Could use torso_arm
//	std::cout << "Model frame: " << kinematic_model->getModelFrame().c_str() << std::endl;
	group.startStateMonitor();

	const robot_state::JointModelGroup* jmg = kinematic_model->getJointModelGroup("arm");

	std::vector<double> group_variable_values;
	group.startStateMonitor();
	group_variable_values = group.getCurrentJointValues();

	kinematic_state->setToRandomPositions(jmg);

	ros::Rate r(20);

//	std::cout << "Starting loop" << std::endl;
	// Don't need to initialise these every time
	Eigen::MatrixXd mJacobian;
	Eigen::Matrix3d mOri;
	Eigen::Vector3d vPos;
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

	while (ros::ok()) {

		if (jointAngles.size() > 0) {

			kinematic_state->setJointGroupPositions(jmg, jointAngles);

//			std::cout << "Group variable values:" << std::endl;

			std::vector<double> joint_values;
/*			std::cout << "Joint values:" << std::endl;
			for(std::size_t i = 0; i < jmg->getLinkModelNames().size(); ++i) {
			    std::cout << "  Joint " << jmg->getJointModelNames()[i].c_str() << ": " << jointAngles[i] << std::endl;
			}
*/

			// Get Jacobian
			kinematic_state->getJacobian(jmg, kinematic_state->getLinkModel(jmg->getLinkModelNames().back()), reference_point_position, mJacobian);
			// Print Jacobian
//			std::cout << "Jacobian:" << std::endl << mJacobian << std::endl;

			// Get forward kinematics
			const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(jmg->getLinkModelNames().back());
			vPos = end_effector_state.translation();
			mOri = end_effector_state.rotation();

			// Print translation and rotation in move_base frame
/*
			std::cout << "Translation: " << std::endl << end_effector_state.translation() << std::endl;
			std::cout << "Rotation:    " << std::endl << end_effector_state.rotation() << std::endl;
*/
			// Build messages
			geometry_msgs::Vector3 msgPosition;		// Position
			msgPosition.x = vPos[0];
			msgPosition.y = vPos[1];
			msgPosition.z = vPos[2];
			std_msgs::Float32MultiArray msgOrientation;	// Orientation (column-major)
			for (unsigned int i = 0; i < 3; ++i) {
				for (unsigned int j = 0; j < 3; ++j) {
					msgOrientation.data.push_back(mOri(i,j));
				}
			}
			std_msgs::Float32MultiArray msgJacobian;	// Jacobian (column-major)
			for (unsigned int i = 0; i < 6; ++i) {
				for (unsigned int j = 0; j < 7; ++j) {
					msgJacobian.data.push_back(mJacobian(i,j));
				}
			}
			// Publish messages
			pubJacobian.publish(msgJacobian);
			pubFKPos.publish(msgPosition);
			pubFKOri.publish(msgOrientation);

		}
		ros::spinOnce();
		r.sleep();
        }

	return EXIT_SUCCESS;
}

