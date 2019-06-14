#ifndef ROSMSG_EIGEN_EXCHANGE
#define ROSMSG_EIGEN_EXCHANGE

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "gen_utilities/XYZ_bxbybz.h"
#include "gen_utilities/Waypoints.h"
#include "gen_utilities/KukaJointAnglesSet.h"

class ros_eigen_exch
{
	// class with static member methods for conversion between ros msgs and eigen matrices
public:
	static Eigen::MatrixXd pose_to_eigen(geometry_msgs::Pose&);
	static Eigen::MatrixXd pose_array_to_eigen(geometry_msgs::PoseArray& );
	static geometry_msgs::Pose eigen_to_pose(Eigen::MatrixXd);
	static geometry_msgs::PoseArray eigen_to_pose_array(Eigen::MatrixXd);
	static Eigen::MatrixXd joint_state_to_eigen(sensor_msgs::JointState&);
	static sensor_msgs::JointState eigen_to_joint_state(Eigen::MatrixXd);
	static Eigen::MatrixXd xyz_bxbybz_to_eigen(gen_utilities::Waypoints&);
	static gen_utilities::Waypoints eigen_to_xyz_bxbybz(Eigen::MatrixXd);
	static Eigen::MatrixXd joint_angles_to_eigen(gen_utilities::KukaJointAnglesSet&);
	static gen_utilities::KukaJointAnglesSet eigen_to_joint_angles(Eigen::MatrixXd);
	static Eigen::MatrixXd group_idx_to_eigen(gen_utilities::KukaJointAnglesSet&);
	static gen_utilities::KukaJointAnglesSet eigen_to_group_idx(Eigen::MatrixXd);

};

#endif