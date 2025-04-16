#include <iostream>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <cassert>
#include "UAVState_mult.h"
#define pi acos(-1)

double N = 4.;
double g_uav = 9.8; // g of uav
double g_tar = 0.;  // g of target on the ground
double etat = 3.;
double etap = 0.025;
double mass_UC = 2.065; // mass of uav and camera (kg)
double thr_max = 10.325;  // max thrust (N)
double delta_t = 0.1;

// ukf 估計的目標物船隻 wamv 資料從下一行的 topic 取用
// /uav0/estimation/ukf/output_data

UAVState::UAVState()
{
	// car_odom_sub = nhdl.subscribe<nav_msgs::Odometry>
    //         ("wamv/base_pose_ground_truth", 10, &UAVState::getAgentOdom, this);

	car_odom_sub = nhdl.subscribe<nav_msgs::Odometry>
            ("/wamv/base_pose_ground_truth", 10, &UAVState::getAgentOdom, this);

	// uav_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
	// 		("mavros/local_position/pose", 1, &UAVState::getUavPos, this);
	// uav_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
	// 		("mavros/local_position/velocity_local", 1, &UAVState::getUavVel, this);
	// uav_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
	// 		("mavros/setpoint_velocity/cmd_vel", 10);
	// uav_cmd_unstamped_pub = nhdl.advertise<geometry_msgs::Twist>
	// 		("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	// uav_cmd_waypint_pub = nhdl.advertise<geometry_msgs::PoseStamped>
	// 		("mavros/setpoint_position/local", 10);
	// uav_cmd_acc_pub = nhdl.advertise<geometry_msgs::Vector3Stamped>
	// 		("mavros/setpoint_accel/accel", 10);
	// uav_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
	// 		("mavros/setpoint_raw/attitude", 10);
	uav0_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
			("uav0/mavros/local_position/pose", 1, &UAVState::getUavPos0, this);
	uav0_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
			("uav0/mavros/local_position/velocity_local", 1, &UAVState::getUavVel0, this);
	uav1_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
			("uav1/mavros/local_position/pose", 1, &UAVState::getUavPos1, this);
	uav1_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
			("uav1/mavros/local_position/velocity_local", 1, &UAVState::getUavVel1, this);
	uav2_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
			("uav2/mavros/local_position/pose", 1, &UAVState::getUavPos2, this);
	uav2_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
			("uav2/mavros/local_position/velocity_local", 1, &UAVState::getUavVel2, this);

	


	uav0_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
			("uav0/mavros/setpoint_velocity/cmd_vel", 10);
	uav1_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
			("uav1/mavros/setpoint_velocity/cmd_vel", 10);
	uav2_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
			("uav2/mavros/setpoint_velocity/cmd_vel", 10);

	uav_cmd_unstamped_pub = nhdl.advertise<geometry_msgs::Twist>
			("uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	uav_cmd_waypint_pub = nhdl.advertise<geometry_msgs::PoseStamped>
			("uav0/mavros/setpoint_position/local", 10);
	uav_cmd_acc_pub = nhdl.advertise<geometry_msgs::Vector3Stamped>
			("uav0/mavros/setpoint_accel/accel", 10);
	uav0_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
			("uav0/mavros/setpoint_raw/attitude", 10);
	uav1_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
			("uav1/mavros/setpoint_raw/attitude", 10);
	uav2_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
			("uav2/mavros/setpoint_raw/attitude", 10);

}

void UAVState::getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom) // message type 記得改
{
	// ground truth
    carVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

	// UKF
	// carVel << odom->target_vel.x, odom->target_vel.y, odom->target_vel.z;

	// ground truth
    carPos.position.x = odom->pose.pose.position.x;
    carPos.position.y = odom->pose.pose.position.y;
    carPos.position.z = odom->pose.pose.position.z;

	// UKF
	// carPos.position.x = odom->target_pose.x;
    // carPos.position.y = odom->target_pose.y;
    // carPos.position.z = odom->target_pose.z;
}

void UAVState::getUavPos0(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	uavPos.position.x = pos->pose.position.x;
	uavPos.position.y = pos->pose.position.y;
	uavPos.position.z = pos->pose.position.z;

	uavPos.orientation.x = pos->pose.orientation.x;
	uavPos.orientation.y = pos->pose.orientation.y;
	uavPos.orientation.z = pos->pose.orientation.z;
	uavPos.orientation.w = pos->pose.orientation.w;

	q_x = uavPos.orientation.x;
	q_y = uavPos.orientation.y;
	q_z = uavPos.orientation.z;
	q_w = uavPos.orientation.w;

	double sinr_cosp = 2. * (q_w * q_x + q_y * q_z);
    double cosr_cosp = 1. - 2. * (q_x * q_x + q_y * q_y);
	phi_ang = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = std::sqrt(1. + 2. * (q_w * q_y - q_x * q_z));
    double cosp = std::sqrt(1. - 2. * (q_w * q_y - q_x * q_z));
	theta_ang = 2. * std::atan2(sinp, cosp) - pi / 2.;

	double siny_cosp = 2. * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1. - 2. * (q_y * q_y + q_z * q_z);
	psi_ang = std::atan2(siny_cosp, cosy_cosp);

	// difference of uav0's position initial condition between Qground and gazebo
	x0 = uavPos.position.x - 400.;
	y0 = uavPos.position.y + 50.;
	z0= uavPos.position.z + 51.;
}

void UAVState::getUavVel0(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	uavLinearVel << vel->twist.linear.x, vel->twist.linear.y, vel->twist.linear.z;
	uavAngularVel << vel->twist.angular.x, vel->twist.angular.y, vel->twist.angular.z;

	vx0 = uavLinearVel[0];
	vy0 = uavLinearVel[1];
	vz0 = uavLinearVel[2];
}

void UAVState::getUavPos1(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	uavPos.position.x = pos->pose.position.x;
	uavPos.position.y = pos->pose.position.y;
	uavPos.position.z = pos->pose.position.z;

	uavPos.orientation.x = pos->pose.orientation.x;
	uavPos.orientation.y = pos->pose.orientation.y;
	uavPos.orientation.z = pos->pose.orientation.z;
	uavPos.orientation.w = pos->pose.orientation.w;

	q_x = uavPos.orientation.x;
	q_y = uavPos.orientation.y;
	q_z = uavPos.orientation.z;
	q_w = uavPos.orientation.w;

	double sinr_cosp = 2. * (q_w * q_x + q_y * q_z);
    double cosr_cosp = 1. - 2. * (q_x * q_x + q_y * q_y);
	phi_ang = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = std::sqrt(1. + 2. * (q_w * q_y - q_x * q_z));
    double cosp = std::sqrt(1. - 2. * (q_w * q_y - q_x * q_z));
	theta_ang = 2. * std::atan2(sinp, cosp) - pi / 2.;

	double siny_cosp = 2. * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1. - 2. * (q_y * q_y + q_z * q_z);
	psi_ang = std::atan2(siny_cosp, cosy_cosp);

	// difference of uav1's position initial condition between Qground and gazebo
	x1 = uavPos.position.x - 400.;
	y1 = uavPos.position.y + 0.;
	z1 = uavPos.position.z + 51.;
}

void UAVState::getUavVel1(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	uavLinearVel << vel->twist.linear.x, vel->twist.linear.y, vel->twist.linear.z;
	uavAngularVel << vel->twist.angular.x, vel->twist.angular.y, vel->twist.angular.z;

	vx1 = uavLinearVel[0];
	vy1 = uavLinearVel[1];
	vz1 = uavLinearVel[2];
}

void UAVState::getUavPos2(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	uavPos.position.x = pos->pose.position.x;
	uavPos.position.y = pos->pose.position.y;
	uavPos.position.z = pos->pose.position.z;

	uavPos.orientation.x = pos->pose.orientation.x;
	uavPos.orientation.y = pos->pose.orientation.y;
	uavPos.orientation.z = pos->pose.orientation.z;
	uavPos.orientation.w = pos->pose.orientation.w;

	q_x = uavPos.orientation.x;
	q_y = uavPos.orientation.y;
	q_z = uavPos.orientation.z;
	q_w = uavPos.orientation.w;

	double sinr_cosp = 2. * (q_w * q_x + q_y * q_z);
    double cosr_cosp = 1. - 2. * (q_x * q_x + q_y * q_y);
	phi_ang = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = std::sqrt(1. + 2. * (q_w * q_y - q_x * q_z));
    double cosp = std::sqrt(1. - 2. * (q_w * q_y - q_x * q_z));
	theta_ang = 2. * std::atan2(sinp, cosp) - pi / 2.;

	double siny_cosp = 2. * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1. - 2. * (q_y * q_y + q_z * q_z);
	psi_ang = std::atan2(siny_cosp, cosy_cosp);

	// difference of uav2's position initial condition between Qground and gazebo
	x2 = uavPos.position.x - 400.;
	y2 = uavPos.position.y - 50.;
	z2 = uavPos.position.z + 51.;
}

void UAVState::getUavVel2(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	uavLinearVel << vel->twist.linear.x, vel->twist.linear.y, vel->twist.linear.z;
	uavAngularVel << vel->twist.angular.x, vel->twist.angular.y, vel->twist.angular.z;

	vx2 = uavLinearVel[0];
	vy2 = uavLinearVel[1];
	vz2 = uavLinearVel[2];
}

void UAVState::showPosVel0()
{
	std::cout << "The position of UAV0 is (" << x0 << ", " << y0 << ", " << z0 << ")" << std::endl;
	std::cout << "The velocity of UAV0 is (" << vx0 << ", " << vy0 << ", " << vz0 << ") = " << sqrt(vx0*vx0+vy0*vy0+vz0*vz0) << "m/s" << std::endl;
	// std::cout << "The vel_thm is (" << vxm_thm << ", " << vym_thm << ", " << vzm_thm << ")" << std::endl;
}

void UAVState::showPosVel1()
{
	std::cout << "The position of UAV1 is (" << x1 << ", " << y1 << ", " << z1 << ")" << std::endl;
	std::cout << "The velocity of UAV1 is (" << vx1 << ", " << vy1 << ", " << vz1 << ") = " << sqrt(vx1*vx1+vy1*vy1+vz1*vz1) << "m/s" << std::endl;
	// std::cout << "The vel_thm is (" << vxm_thm << ", " << vym_thm << ", " << vzm_thm << ")" << std::endl;
}

void UAVState::showPosVel2()
{
	std::cout << "The position of UAV is (" << x2 << ", " << y2 << ", " << z2 << ")" << std::endl;
	std::cout << "The velocity of UAV is (" << vx2 << ", " << vy2 << ", " << vz2 << ") = " << sqrt(vx2*vx2+vy2*vy2+vz2*vz2) << "m/s" << std::endl;
	// std::cout << "The vel_thm is (" << vxm_thm << ", " << vym_thm << ", " << vzm_thm << ")" << std::endl;
}

double UAVState::get_dis_waypoint0(double x_wp, double y_wp, double z_wp)
{
	double xd, yd, zd, distance;

	xd = x0 - x_wp;
	yd = y0 - y_wp;
	zd = z0 - z_wp;

	// distance = sqrt(xd*xd + yd*yd + zd*zd);
	distance = sqrt(xd*xd + yd*yd);

	return (distance);
}

double UAVState::get_dis_waypoint1(double x_wp, double y_wp, double z_wp)
{
	double xd, yd, zd, distance;

	xd = x1 - x_wp;
	yd = y1 - y_wp;
	zd = z1 - z_wp;

	// distance = sqrt(xd*xd + yd*yd + zd*zd);
	distance = sqrt(xd*xd + yd*yd);

	return (distance);
}

double UAVState::get_dis_waypoint2(double x_wp, double y_wp, double z_wp)
{
	double xd, yd, zd, distance;

	xd = x2 - x_wp;
	yd = y2 - y_wp;
	zd = z2 - z_wp;

	// distance = sqrt(xd*xd + yd*yd + zd*zd);
	distance = sqrt(xd*xd + yd*yd);

	return (distance);
}

void UAVState::vel_cmd0(double vx, double vy, double vz)
{
	uavLinearVelCmd.twist.linear.x = vx;
	uavLinearVelCmd.twist.linear.y = vy;
	uavLinearVelCmd.twist.linear.z = vz;

	uav0_cmd_vel_pub.publish(uavLinearVelCmd);
	std::cout << "The vel_cmd is (" << uavLinearVelCmd.twist.linear.x << ", " << uavLinearVelCmd.twist.linear.y << ", " << uavLinearVelCmd.twist.linear.z << ")" << std::endl;

	ros::spinOnce();
    rate.sleep();
}

void UAVState::vel_cmd1(double vx, double vy, double vz)
{
	uavLinearVelCmd.twist.linear.x = vx;
	uavLinearVelCmd.twist.linear.y = vy;
	uavLinearVelCmd.twist.linear.z = vz;

	uav1_cmd_vel_pub.publish(uavLinearVelCmd);
	std::cout << "The vel_cmd is (" << uavLinearVelCmd.twist.linear.x << ", " << uavLinearVelCmd.twist.linear.y << ", " << uavLinearVelCmd.twist.linear.z << ")" << std::endl;

	ros::spinOnce();
    rate.sleep();
}

void UAVState::vel_cmd2(double vx, double vy, double vz)
{
	uavLinearVelCmd.twist.linear.x = vx;
	uavLinearVelCmd.twist.linear.y = vy;
	uavLinearVelCmd.twist.linear.z = vz;

	uav2_cmd_vel_pub.publish(uavLinearVelCmd);
	std::cout << "The vel_cmd is (" << uavLinearVelCmd.twist.linear.x << ", " << uavLinearVelCmd.twist.linear.y << ", " << uavLinearVelCmd.twist.linear.z << ")" << std::endl;

	ros::spinOnce();
    rate.sleep();
}

void UAVState::bpng_acc_cmd0(double varia, double varie)
{
	// For other points except initial condition
	double xm = y0, ym = x0, zm = z0; // xm,ym,and zm are for bpnlaw in matlab
	double vxm = vy0, vym = vx0, vzm = vz0; // vxm, vym, and vzm are for bpnlaw in matlab
	double xt = carPos.position.y, yt = carPos.position.x, zt = carPos.position.z; // xt, yt, and zt are for bpnlaw in matlab 
	double vxt = carVel[1], vyt = carVel[0], vzt = carVel[2]; // vxt, vyt, and vzt are for bpnlaw in matlab
	double rx, ry, rz, vrex,vrey, vrez, rxy, r; // x is the x vector of UAV position, and xt is the x vector of target
	double r1, r2, r3, v1, v2, v3;
	double vclxy,vclxy_r, vcl, vcl_r, tgo;
	double aLOS, agammat, agamma, athet, athe, av, avt, adLOS, adesireLOS, adbias;
	double vl, vl_r, rl, vtl, vtl_r;
	double eLOS, egammat, egamma, ethet, ethe, ev, ev_r, evt, evt_r, edLOS, edesireLOS, edbias;
	double predict_evt, predict_egammat, predict_ev;
	double Aca, Ace, Acla, Acle, axc, ayc, azc;
	double acc_h_x, acc_h_y, acc_h_z;
	double roll_ang, pitch_ang, yaw_ang;
	double q_w, q_x, q_y, q_z;
	double thrust, thr;
	double acc_sum, acc_h_sum;
	double axc_enu, ayc_enu, azc_enu;
	double xm_next, ym_next, zm_next, vx_next, vy_next, vz_next, v_next, unit_vx_next, unit_vy_next, unit_vz_next, thetaa;
	double roll_rate, pitch_rate, yaw_rate, v_total;

	std::cout << "[uav0]" << "\n";
	std::cout << "The position of the UAV0 is (" << x0 << ", " << y0 << ", " << z0 << ")" << std::endl;
	std::cout << "The velocity of the UAV0 is (" << vy0 << ", " << vx0 << ", " << vz0 << ")" << std::endl;
	std::cout << "The position of the boat is (" << (carPos.position.x - 25.)<< ", " << carPos.position.y << ", " << (carPos.position.z + 18.) << ")" << std::endl;
	std::cout << "The velocity of the boat is (" << carVel[0] << ", " << carVel[1] << ", " << carVel[2] << ")" << std::endl;

	// relative position and velocity
	rx = xt - xm; 
	ry = yt - ym;
	rz = zt - zm;
	vrex = vxt - vxm; 
	vrey = vyt - vym;
	vrez = vzt - vzm;
	rxy = sqrt(rx * rx + ry * ry);
	r = sqrt(rx * rx + ry * ry + rz * rz);

	if (r <= 0.01) {
		r = 0.01;
	}
	if (rxy <= 0.01) {
		rxy = 0.01;
	}
	if (r <= 10){
		this->is_hit[0];
	}

	// change of coordinates
	r1 = ry;
	r2 = rx;
	r3 = -rz;
	v1 = vrey;
	v2 = vrex;
	v3 = -vrez;

	// closing velocity
	vclxy = -(r1 * v1 + r2 * v2) / rxy;
	vcl = -(r1 * v1 + r2 * v2 + r3 * v3) / r;

	// time to go
	tgo = r / vcl;

	// azimuth part

	aLOS = atan2(ry, rx); // azimuth line of sight
	agammat = atan2(vyt, vxt);
	agamma = atan2(vym, vxm);

	// azimuth angle theta of target and UAV
	athet = agammat - aLOS;
	athe = agamma - aLOS;

	// azimuth velocity of UAV and target
	av = sqrt(vxm * vxm + vym * vym);
	avt = sqrt(vxt * vxt + vyt * vyt);

	adLOS = (avt * sin(athet) - av * sin(athe)) / rxy;
	if (athet>pi || athet<0) {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	} else {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat + varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat + varia * pi / 180.)));
	}
	// adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	// std::cout << "The adesireLOS is " << adesireLOS * 180. / pi << std::endl;
	adbias = ((etat * vclxy * (adesireLOS - aLOS)) / (N * rxy));

	// elevation part
	// vl = sqrt((vxm * vxm) + (vym * vym));
	vl_r = av*cos(athe); // 投影到R垂直面
	// rl = sqrt((rx * rx) + (ry * ry));
	// vtl = -sqrt((vxt * vxt) + (vyt * vyt));
	vtl_r = avt*cos(athet); // 投影到R垂直面

	eLOS = atan2(rz, rxy);
	egamma = atan2(vzm, vl_r);
	egammat = atan2(vzt, vtl_r);
	ethet = egammat - eLOS;
	ethe = egamma - eLOS;

	vclxy_r = sqrt(vxm*vxm + vym*vym) * cos(agamma);
	vcl_r = sqrt(vxm*vxm + vym*vym + vzm*vzm) * cos(agamma);

	ev = sqrt(vzm*vzm + av*av);
	ev_r = sqrt(vzm*vzm + vl_r*vl_r);
	evt_r = sqrt(vzt*vzt + vtl_r*vtl_r);

	edLOS = (evt_r * sin(ethet) - ev_r * sin(ethe)) / r;

	// consider gravity
	predict_evt = abs(evt_r - tgo * g_tar * sin(egammat));
	predict_egammat = egammat - tgo * g_tar * cos(egammat) / predict_evt;
	predict_ev = abs(ev_r - tgo * g_uav * sin(egamma));

	if (ethet>pi || ethet<0) {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat + varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat + varie * pi / 180.)));
	} else {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));
	}
	// edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));

	edbias = ((etat * vcl * (edesireLOS - eLOS)) / (N * r));

	// acceleration command
	if (r >= 10.) {
		// PN law
		Aca = N * vclxy * adLOS;
		Ace = N * vcl * edLOS;
		// bpn 1.0
		// Aca = N * vclxy * adLOS - etap * rxy * (adesireLOS - aLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS) - g_uav*cos(egamma);
		// bpn 2.0 
		// Aca = N*vclxy*(adLOS-adbias);
		// Ace = N*vcl*(edLOS-edbias);

		Acla = Aca;
		Acle = Ace;

		axc = -Ace * cos(aLOS) * sin(eLOS) - Aca * sin(aLOS);
		ayc = -Ace * sin(aLOS) * sin(eLOS) + Aca * cos(aLOS);
		azc = Ace * cos(eLOS);
	}
	else {
		axc = -Acle * cos(aLOS) * sin(eLOS) - Acla * sin(aLOS);
		ayc = -Acle * sin(aLOS) * sin(eLOS) + Acla * cos(aLOS);
		azc = Acle * cos(eLOS);
		Aca = Acla;
		Ace = Acle;
	}

	acc_sum = sqrt(axc*axc+ayc*ayc);
	// std::cout << "The acc_cmd in gazebo frame is (" << ayc << ", " << axc << ", " << azc << ") = " << acc_sum << " m/s^2" << std::endl;
	std::cout << "The acc_azimuth is " << Aca << ", and the acc_elevation is " << Ace << "m/s^2" << std::endl;

	////////////////////////////////////////////////////////////////////////////
	// Publish the angular velocity and thrust

	// calculate pitch rate Q and yaw rate R
	v_total = sqrt(vxm*vxm + vym*vym + vzm*vzm);
	pitch_rate = -Ace / v_total;
	yaw_rate = Aca / v_total;

	double phi_bar;
	phi_bar = std::atan2(v_total*yaw_rate, 9.81);
	double k_roll_rate = 2.;
	roll_rate = k_roll_rate*(phi_bar - phi_ang);
	int mask = 132;
	std::cout << "The roll rate = " << roll_rate << ", and pitch rate = " << pitch_rate << ", and yaw rate = " << yaw_rate << std::endl;
	// std::cout << "The mask is " << mask << "\n" << std::endl;

	// P control velocity
	double thr_cmd, v_ref = 33., v_diff;
	v_diff = v_ref - v_total;
	if (v_diff > 0.) {
		thr_cmd = 2.*v_diff/v_ref;
	} else {
		thr_cmd = 0.;
	}
	std::cout << "The thrust command is " << thr_cmd << "\n" << std::endl;

	uavAttitudeThrustCmd.type_mask = mask;
	uavAttitudeThrustCmd.orientation.x = q_x;
	uavAttitudeThrustCmd.orientation.y = q_y;
	uavAttitudeThrustCmd.orientation.z = q_z;
	uavAttitudeThrustCmd.orientation.w = q_w;
	uavAttitudeThrustCmd.body_rate.x = roll_rate;
	uavAttitudeThrustCmd.body_rate.y = pitch_rate;
	uavAttitudeThrustCmd.body_rate.z = yaw_rate;
	uavAttitudeThrustCmd.thrust = thr_cmd;

	// uavAttitudeThrustCmd.body_rate.x = 0;
	// uavAttitudeThrustCmd.body_rate.y = 0.1;
	// uavAttitudeThrustCmd.body_rate.z = 0;


	uav0_cmd_att_thr_pub.publish(uavAttitudeThrustCmd);
}

void UAVState::bpng_acc_cmd1(double varia, double varie)
{
	// For other points except initial condition
	double xm = y1, ym = x1, zm = z1; // xm,ym,and zm are for bpnlaw in matlab
	double vxm = vy1, vym = vx1, vzm = vz1; // vxm, vym, and vzm are for bpnlaw in matlab
	double xt = carPos.position.y, yt = carPos.position.x, zt = carPos.position.z; // xt, yt, and zt are for bpnlaw in matlab 
	double vxt = carVel[1], vyt = carVel[0], vzt = carVel[2]; // vxt, vyt, and vzt are for bpnlaw in matlab
	double rx, ry, rz, vrex,vrey, vrez, rxy, r; // x is the x vector of UAV position, and xt is the x vector of target
	double r1, r2, r3, v1, v2, v3;
	double vclxy,vclxy_r, vcl, vcl_r, tgo;
	double aLOS, agammat, agamma, athet, athe, av, avt, adLOS, adesireLOS, adbias;
	double vl, vl_r, rl, vtl, vtl_r;
	double eLOS, egammat, egamma, ethet, ethe, ev, ev_r, evt, evt_r, edLOS, edesireLOS, edbias;
	double predict_evt, predict_egammat, predict_ev;
	double Aca, Ace, Acla, Acle, axc, ayc, azc;
	double acc_h_x, acc_h_y, acc_h_z;
	double roll_ang, pitch_ang, yaw_ang;
	double q_w, q_x, q_y, q_z;
	double thrust, thr;
	double acc_sum, acc_h_sum;
	double axc_enu, ayc_enu, azc_enu;
	double xm_next, ym_next, zm_next, vx_next, vy_next, vz_next, v_next, unit_vx_next, unit_vy_next, unit_vz_next, thetaa;
	double roll_rate, pitch_rate, yaw_rate, v_total;

	std::cout << "[uav1]" << "\n";
	std::cout << "The position of the UAV1 is (" << x1 << ", " << y1 << ", " << z1 << ")" << std::endl;
	std::cout << "The velocity of the UAV1 is (" << vx1 << ", " << vy1 << ", " << vz1 << ")" << std::endl;
	std::cout << "The position of the boat is (" << (carPos.position.x - 25.)<< ", " << carPos.position.y << ", " << (carPos.position.z + 18.) << ")" << std::endl;
	std::cout << "The velocity of the boat is (" << carVel[0] << ", " << carVel[1] << ", " << carVel[2] << ")" << std::endl;

	// relative position and velocity
	rx = xt - xm; 
	ry = yt - ym;
	rz = zt - zm;
	vrex = vxt - vxm; 
	vrey = vyt - vym;
	vrez = vzt - vzm;
	rxy = sqrt(rx * rx + ry * ry);
	r = sqrt(rx * rx + ry * ry + rz * rz);

	if (r <= 0.01) {
		r = 0.01;
	}
	if (rxy <= 0.01) {
		rxy = 0.01;
	}
	if (r <= 10){
		this->is_hit[1];
	}

	// change of coordinates
	r1 = ry;
	r2 = rx;
	r3 = -rz;
	v1 = vrey;
	v2 = vrex;
	v3 = -vrez;

	// closing velocity
	vclxy = -(r1 * v1 + r2 * v2) / rxy;
	vcl = -(r1 * v1 + r2 * v2 + r3 * v3) / r;

	// time to go
	tgo = r / vcl;

	// azimuth part

	aLOS = atan2(ry, rx); // azimuth line of sight
	agammat = atan2(vyt, vxt);
	agamma = atan2(vym, vxm);

	// azimuth angle theta of target and UAV
	athet = agammat - aLOS;
	athe = agamma - aLOS;

	// azimuth velocity of UAV and target
	av = sqrt(vxm * vxm + vym * vym);
	avt = sqrt(vxt * vxt + vyt * vyt);

	adLOS = (avt * sin(athet) - av * sin(athe)) / rxy;
	if (athet>pi || athet<0) {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	} else {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat + varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat + varia * pi / 180.)));
	}
	// adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	// std::cout << "The adesireLOS is " << adesireLOS * 180. / pi << std::endl;
	adbias = ((etat * vclxy * (adesireLOS - aLOS)) / (N * rxy));

	// elevation part
	// vl = sqrt((vxm * vxm) + (vym * vym));
	vl_r = av*cos(athe); // 投影到R垂直面
	// rl = sqrt((rx * rx) + (ry * ry));
	// vtl = -sqrt((vxt * vxt) + (vyt * vyt));
	vtl_r = avt*cos(athet); // 投影到R垂直面

	eLOS = atan2(rz, rxy);
	egamma = atan2(vzm, vl_r);
	egammat = atan2(vzt, vtl_r);
	ethet = egammat - eLOS;
	ethe = egamma - eLOS;

	vclxy_r = sqrt(vxm*vxm + vym*vym) * cos(agamma);
	vcl_r = sqrt(vxm*vxm + vym*vym + vzm*vzm) * cos(agamma);

	ev = sqrt(vzm*vzm + av*av);
	ev_r = sqrt(vzm*vzm + vl_r*vl_r);
	evt_r = sqrt(vzt*vzt + vtl_r*vtl_r);

	edLOS = (evt_r * sin(ethet) - ev_r * sin(ethe)) / r;

	// consider gravity
	predict_evt = abs(evt_r - tgo * g_tar * sin(egammat));
	predict_egammat = egammat - tgo * g_tar * cos(egammat) / predict_evt;
	predict_ev = abs(ev_r - tgo * g_uav * sin(egamma));

	if (ethet>pi || ethet<0) {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat + varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat + varie * pi / 180.)));
	} else {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));
	}
	// edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));

	edbias = ((etat * vcl * (edesireLOS - eLOS)) / (N * r));

	// acceleration command
	if (r >= 10.) {
		// PN law
		Aca = N * vclxy * adLOS;
		Ace = N * vcl * edLOS;
		// bpn 1.0
		// Aca = N * vclxy * adLOS - etap * rxy * (adesireLOS - aLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS) - g_uav*cos(egamma);
		// bpn 2.0
		// Aca = N*vclxy*(adLOS-adbias);
		// Ace = N*vcl*(edLOS-edbias);

		Acla = Aca;
		Acle = Ace;

		axc = -Ace * cos(aLOS) * sin(eLOS) - Aca * sin(aLOS);
		ayc = -Ace * sin(aLOS) * sin(eLOS) + Aca * cos(aLOS);
		azc = Ace * cos(eLOS);
	}
	else {
		axc = -Acle * cos(aLOS) * sin(eLOS) - Acla * sin(aLOS);
		ayc = -Acle * sin(aLOS) * sin(eLOS) + Acla * cos(aLOS);
		azc = Acle * cos(eLOS);
		Aca = Acla;
		Ace = Acle;
	}

	acc_sum = sqrt(axc*axc+ayc*ayc);
	// std::cout << "The acc_cmd in gazebo frame is (" << ayc << ", " << axc << ", " << azc << ") = " << acc_sum << " m/s^2" << std::endl;
	std::cout << "The acc_azimuth is " << Aca << ", and the acc_elevation is " << Ace << "m/s^2" << std::endl;

	////////////////////////////////////////////////////////////////////////////
	// Publish the angular velocity and thrust

	// calculate pitch rate Q and yaw rate R
	v_total = sqrt(vxm*vxm + vym*vym + vzm*vzm);
	pitch_rate = -Ace / v_total;
	yaw_rate = Aca / v_total;

	double phi_bar;
	phi_bar = std::atan2(v_total*yaw_rate, 9.81);
	double k_roll_rate = 2.;
	roll_rate = k_roll_rate*(phi_bar - phi_ang);
	int mask = 132;
	std::cout << "The roll rate = " << roll_rate << ", and pitch rate = " << pitch_rate << ", and yaw rate = " << yaw_rate << std::endl;
	// std::cout << "The mask is " << mask << "\n" << std::endl;

	// P control velocity
	double thr_cmd, v_ref = 33., v_diff;
	v_diff = v_ref - v_total;
	if (v_diff > 0.) {
		thr_cmd = 2.*v_diff/v_ref;
	} else {
		thr_cmd = 0.;
	}
	std::cout << "The thrust command is " << thr_cmd << "\n" << std::endl;

	uavAttitudeThrustCmd.type_mask = mask;
	uavAttitudeThrustCmd.orientation.x = q_x;
	uavAttitudeThrustCmd.orientation.y = q_y;
	uavAttitudeThrustCmd.orientation.z = q_z;
	uavAttitudeThrustCmd.orientation.w = q_w;
	uavAttitudeThrustCmd.body_rate.x = roll_rate;
	uavAttitudeThrustCmd.body_rate.y = pitch_rate;
	uavAttitudeThrustCmd.body_rate.z = yaw_rate;
	uavAttitudeThrustCmd.thrust = thr_cmd;

	// uavAttitudeThrustCmd.body_rate.x = 0;
	// uavAttitudeThrustCmd.body_rate.y = 0.1;
	// uavAttitudeThrustCmd.body_rate.z = 0;


	uav1_cmd_att_thr_pub.publish(uavAttitudeThrustCmd);
}

void UAVState::bpng_acc_cmd2(double varia, double varie)
{
	// For other points except initial condition
	double xm = y2, ym = x2, zm = z2; // xm,ym,and zm are for bpnlaw in matlab
	double vxm = vy2, vym = vx2, vzm = vz2; // vxm, vym, and vzm are for bpnlaw in matlab
	double xt = carPos.position.y, yt = carPos.position.x, zt = carPos.position.z; // xt, yt, and zt are for bpnlaw in matlab 
	double vxt = carVel[1], vyt = carVel[0], vzt = carVel[2]; // vxt, vyt, and vzt are for bpnlaw in matlab
	double rx, ry, rz, vrex,vrey, vrez, rxy, r; // x is the x vector of UAV position, and xt is the x vector of target
	double r1, r2, r3, v1, v2, v3;
	double vclxy,vclxy_r, vcl, vcl_r, tgo;
	double aLOS, agammat, agamma, athet, athe, av, avt, adLOS, adesireLOS, adbias;
	double vl, vl_r, rl, vtl, vtl_r;
	double eLOS, egammat, egamma, ethet, ethe, ev, ev_r, evt, evt_r, edLOS, edesireLOS, edbias;
	double predict_evt, predict_egammat, predict_ev;
	double Aca, Ace, Acla, Acle, axc, ayc, azc;
	double acc_h_x, acc_h_y, acc_h_z;
	double roll_ang, pitch_ang, yaw_ang;
	double q_w, q_x, q_y, q_z;
	double thrust, thr;
	double acc_sum, acc_h_sum;
	double axc_enu, ayc_enu, azc_enu;
	double xm_next, ym_next, zm_next, vx_next, vy_next, vz_next, v_next, unit_vx_next, unit_vy_next, unit_vz_next, thetaa;
	double roll_rate, pitch_rate, yaw_rate, v_total;

	std::cout << "[uav2]" << "\n";
	std::cout << "The position of the UAV2 is (" << x2 << ", " << y2 << ", " << z2 << ")" << std::endl;
	std::cout << "The velocity of the UAV2 is (" << vx2 << ", " << vy2 << ", " << vz2 << ")" << std::endl;
	std::cout << "The position of the boat is (" << (carPos.position.x - 25.)<< ", " << carPos.position.y << ", " << (carPos.position.z + 18.) << ")" << std::endl;
	std::cout << "The velocity of the boat is (" << carVel[0] << ", " << carVel[1] << ", " << carVel[2] << ")" << std::endl;

	// relative position and velocity
	rx = xt - xm; 
	ry = yt - ym;
	rz = zt - zm;
	vrex = vxt - vxm; 
	vrey = vyt - vym;
	vrez = vzt - vzm;
	rxy = sqrt(rx * rx + ry * ry);
	r = sqrt(rx * rx + ry * ry + rz * rz);

	if (r <= 0.01) {
		r = 0.01;
	}
	if (rxy <= 0.01) {
		rxy = 0.01;
	}
	if (r <= 10){
		this->is_hit[2];
	}

	// change of coordinates
	r1 = ry;
	r2 = rx;
	r3 = -rz;
	v1 = vrey;
	v2 = vrex;
	v3 = -vrez;

	// closing velocity
	vclxy = -(r1 * v1 + r2 * v2) / rxy;
	vcl = -(r1 * v1 + r2 * v2 + r3 * v3) / r;

	// time to go
	tgo = r / vcl;

	// azimuth part

	aLOS = atan2(ry, rx); // azimuth line of sight
	agammat = atan2(vyt, vxt);
	agamma = atan2(vym, vxm);

	// azimuth angle theta of target and UAV
	athet = agammat - aLOS;
	athe = agamma - aLOS;

	// azimuth velocity of UAV and target
	av = sqrt(vxm * vxm + vym * vym);
	avt = sqrt(vxt * vxt + vyt * vyt);

	adLOS = (avt * sin(athet) - av * sin(athe)) / rxy;
	if (athet>pi || athet<0) {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	} else {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat + varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat + varia * pi / 180.)));
	}
	// adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	// std::cout << "The adesireLOS is " << adesireLOS * 180. / pi << std::endl;
	adbias = ((etat * vclxy * (adesireLOS - aLOS)) / (N * rxy));

	// elevation part
	// vl = sqrt((vxm * vxm) + (vym * vym));
	vl_r = av*cos(athe); // 投影到R垂直面
	// rl = sqrt((rx * rx) + (ry * ry));
	// vtl = -sqrt((vxt * vxt) + (vyt * vyt));
	vtl_r = avt*cos(athet); // 投影到R垂直面

	eLOS = atan2(rz, rxy);
	egamma = atan2(vzm, vl_r);
	egammat = atan2(vzt, vtl_r);
	ethet = egammat - eLOS;
	ethe = egamma - eLOS;

	vclxy_r = sqrt(vxm*vxm + vym*vym) * cos(agamma);
	vcl_r = sqrt(vxm*vxm + vym*vym + vzm*vzm) * cos(agamma);

	ev = sqrt(vzm*vzm + av*av);
	ev_r = sqrt(vzm*vzm + vl_r*vl_r);
	evt_r = sqrt(vzt*vzt + vtl_r*vtl_r);

	edLOS = (evt_r * sin(ethet) - ev_r * sin(ethe)) / r;

	// consider gravity
	predict_evt = abs(evt_r - tgo * g_tar * sin(egammat));
	predict_egammat = egammat - tgo * g_tar * cos(egammat) / predict_evt;
	predict_ev = abs(ev_r - tgo * g_uav * sin(egamma));

	if (ethet>pi || ethet<0) {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat + varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat + varie * pi / 180.)));
	} else {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));
	}
	// edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));

	edbias = ((etat * vcl * (edesireLOS - eLOS)) / (N * r));

	// acceleration command
	if (r >= 10.) {
		// PN law
		Aca = N * vclxy * adLOS;
		Ace = N * vcl * edLOS;
		// bpn 2.0
		// Aca = N * vclxy * adLOS - etap * rxy * (adesireLOS - aLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS) - g_uav*cos(egamma);
		// bpn 1.0
		// Aca = N*vclxy*(adLOS-adbias);
		// Ace = N*vcl*(edLOS-edbias);

		Acla = Aca;
		Acle = Ace;

		axc = -Ace * cos(aLOS) * sin(eLOS) - Aca * sin(aLOS);
		ayc = -Ace * sin(aLOS) * sin(eLOS) + Aca * cos(aLOS);
		azc = Ace * cos(eLOS);
	}
	else {
		axc = -Acle * cos(aLOS) * sin(eLOS) - Acla * sin(aLOS);
		ayc = -Acle * sin(aLOS) * sin(eLOS) + Acla * cos(aLOS);
		azc = Acle * cos(eLOS);
		Aca = Acla;
		Ace = Acle;
	}

	acc_sum = sqrt(axc*axc+ayc*ayc);
	// std::cout << "The acc_cmd in gazebo frame is (" << ayc << ", " << axc << ", " << azc << ") = " << acc_sum << " m/s^2" << std::endl;
	std::cout << "The acc_azimuth is " << Aca << ", and the acc_elevation is " << Ace << "m/s^2" << std::endl;

	////////////////////////////////////////////////////////////////////////////
	// Publish the angular velocity and thrust

	// calculate pitch rate Q and yaw rate R
	v_total = sqrt(vxm*vxm + vym*vym + vzm*vzm);
	pitch_rate = -Ace / v_total;
	yaw_rate = Aca / v_total;

	double phi_bar;
	phi_bar = std::atan2(v_total*yaw_rate, 9.81);
	double k_roll_rate = 2.;
	roll_rate = k_roll_rate*(phi_bar - phi_ang);
	int mask = 132;
	std::cout << "The roll rate = " << roll_rate << ", and pitch rate = " << pitch_rate << ", and yaw rate = " << yaw_rate << std::endl;
	// std::cout << "The mask is " << mask << "\n" << std::endl;

	// P control velocity
	double thr_cmd, v_ref = 33., v_diff;
	v_diff = v_ref - v_total;
	if (v_diff > 0.) {
		thr_cmd = 2.*v_diff/v_ref;
	} else {
		thr_cmd = 0.;
	}
	std::cout << "The thrust command is " << thr_cmd << "\n" << std::endl;

	uavAttitudeThrustCmd.type_mask = mask;
	uavAttitudeThrustCmd.orientation.x = q_x;
	uavAttitudeThrustCmd.orientation.y = q_y;
	uavAttitudeThrustCmd.orientation.z = q_z;
	uavAttitudeThrustCmd.orientation.w = q_w;
	uavAttitudeThrustCmd.body_rate.x = roll_rate;
	uavAttitudeThrustCmd.body_rate.y = pitch_rate;
	uavAttitudeThrustCmd.body_rate.z = yaw_rate;
	uavAttitudeThrustCmd.thrust = thr_cmd;


	// uavAttitudeThrustCmd.body_rate.x =  0;
	// uavAttitudeThrustCmd.body_rate.y = 0.1;
	// uavAttitudeThrustCmd.body_rate.z = 0;


	uav2_cmd_att_thr_pub.publish(uavAttitudeThrustCmd);
}