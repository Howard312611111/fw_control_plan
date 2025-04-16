/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#ifndef FWCONTROL_HPP
#define FWCONTROL_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

class FwControl
{
private:
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate = 20.0;
    ros::Subscriber state_sub;
    ros::Subscriber global_pos_sub;
    ros::Publisher vel_pub;
    ros::Publisher localPos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::Publisher att_thr_pub;
    ros::Time last_request;
    

    
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool cmd_arm;
    mavros_msgs::State current_state;
    mavros_msgs::CommandTOL cmd_takeoff;
    sensor_msgs::NavSatFix current_lla;
    geometry_msgs::TwistStamped cmd_vel;
    geometry_msgs::PoseStamped cmd_pos;
    mavros_msgs::AttitudeTarget uavAttitudeThrustCmd;

    std::string controlMode = "OFFBOARD";
    bool isTakeoff = 0;


public: 
    FwControl();
    FwControl(int uavID);

    ~FwControl();
    bool CheckFCUConnection();
    bool Initialize();
    bool Initialize2();
    void SwitchMode();
    void SwitchMode2();
    void Takeoff();
    void TrackCircle(std::vector<float> ctr, float radius, float freq);
    void TrackCircle2(std::vector<float> ctr, float radius, float freq);
    void Vel_cmd(double vx, double vy, double vz);
    void Att_Thr_cmd(double rate_x, double rate_y, double rate_z, double thr_cmd);

    void getCurrentState(const mavros_msgs::State::ConstPtr& state);
    void getGlobalLLA(const sensor_msgs::NavSatFix::ConstPtr& lla);
};

#endif