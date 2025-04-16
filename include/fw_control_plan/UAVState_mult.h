#pragma once
// BPNG_3D
#ifndef _UAVSTATE_H_20230801
#define	_UAVSTATE_H_20230801

#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <fw_control/EstimateOutput.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/MountControl.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/AttitudeTarget.h>


///////////////////////////////////////////////////////////////////////////////
// UAV state
///////////////////////////////////////////////////////////////////////////////
class UAVState
{
private:
	double x0, y0, z0;
	double x1, y1, z1;
	double x2, y2, z2;
	double vx0, vy0, vz0;
	double vx1, vy1, vz1;
	double vx2, vy2, vz2;
	double q_x, q_y, q_z, q_w;
	double phi_ang, theta_ang, psi_ang;
	double xm_thm=0, ym_thm=0, zm_thm=0;
	double vxm_thm=0, vym_thm=0, vzm_thm=0;
	ros::NodeHandle nhdl;
	ros::Rate rate = 20.0;

	// publisher and subscriber
	ros::Subscriber car_odom_sub;
	ros::Subscriber uav0_pose_sub;
	ros::Subscriber uav0_velocity_body_sub;
	ros::Subscriber uav1_pose_sub;
	ros::Subscriber uav1_velocity_body_sub;
	ros::Subscriber uav2_pose_sub;
	ros::Subscriber uav2_velocity_body_sub;
	ros::Publisher  uav_cmd_waypint_pub;
	ros::Publisher  uav0_cmd_vel_pub;
	ros::Publisher  uav1_cmd_vel_pub;
	ros::Publisher  uav2_cmd_vel_pub;
	ros::Publisher  uav_cmd_unstamped_pub;
	ros::Publisher  uav_cmd_acc_pub;
	ros::Publisher  uav0_cmd_att_thr_pub;
	ros::Publisher  uav1_cmd_att_thr_pub;
	ros::Publisher  uav2_cmd_att_thr_pub;

	Eigen::Vector3f carVel;
	geometry_msgs::Pose carPos;
	Eigen::Vector3f uavLinearVel;
	Eigen::Vector3f uavAngularVel;
	geometry_msgs::Pose uavPos;
	geometry_msgs::TwistStamped uavLinearVelCmd;
	geometry_msgs::Twist uavLinearVelCmdUstamped;
	geometry_msgs::PoseStamped uavWaypointCmd;
	geometry_msgs::Vector3Stamped uavLinearAccCmd;
	mavros_msgs::AttitudeTarget uavAttitudeThrustCmd;


public:

	UAVState();  // constructor
	~UAVState() {}; // destructor

	void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
	void getUavPos0(const geometry_msgs::PoseStamped::ConstPtr& pos);
	void getUavVel0(const geometry_msgs::TwistStamped::ConstPtr& vel);
	void getUavPos1(const geometry_msgs::PoseStamped::ConstPtr& pos);
	void getUavVel1(const geometry_msgs::TwistStamped::ConstPtr& vel);
	void getUavPos2(const geometry_msgs::PoseStamped::ConstPtr& pos);
	void getUavVel2(const geometry_msgs::TwistStamped::ConstPtr& vel);

	void showPosVel0();
	void showPosVel1();
	void showPosVel2();

	double get_dis_waypoint0(double x_wp, double y_wp, double z_wp);
	double get_dis_waypoint1(double x_wp, double y_wp, double z_wp);
	double get_dis_waypoint2(double x_wp, double y_wp, double z_wp);

	void waypoint_cmd(double x_wp, double y_wp, double z_wp);
	void vel_cmd0(double vx, double vy, double vz);
	void vel_cmd1(double vx, double vy, double vz);
	void vel_cmd2(double vx, double vy, double vz);

	void test_raw_attitude(double vxm, double vym, double vzm);
	void bpng_azi_acc_cmd(double vari);
	void bpng_acc_cmd0(double varia, double varie);
	void bpng_acc_cmd1(double varia, double varie);
	void bpng_acc_cmd2(double varia, double varie);
	
	std::vector<bool> is_hit = {false, false, false};
};



#endif _UAVSTATE_H_20230801