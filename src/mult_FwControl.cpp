#include "mult_FwControl.hpp"

FwControl::FwControl()
{
    // state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, &FwControl::getCurrentState, this);
    // global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
    //         ("mavros/global_position/global", 10, &FwControl::getGlobalLLA, this);
    // vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);
    // localPos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
    // takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
    //         ("mavros/cmd/takeoff");
    // att_thr_pub = nh.advertise<mavros_msgs::AttitudeTarget>
	// 		("mavros/setpoint_raw/attitude", 10);
    
    // state_sub = nh.subscribe<mavros_msgs::State>
    //         ("uav0/mavros/state", 10, &FwControl::getCurrentState, this);
    // global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
    //         ("uav0/mavros/global_position/global", 10, &FwControl::getGlobalLLA, this);
    // vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("uav0/mavros/setpoint_velocity/cmd_vel", 10);
    // localPos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("uav0/mavros/setpoint_position/local", 10);
    // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("uav0/mavros/cmd/arming");
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("uav0/mavros/set_mode");
    // takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
    //         ("uav0/mavros/cmd/takeoff");
    // att_thr_pub = nh.advertise<mavros_msgs::AttitudeTarget>
	// 		("uav0/mavros/setpoint_raw/attitude", 10);

}

FwControl::FwControl(int uavID)
{
    // state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, &FwControl::getCurrentState, this);
    // global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
    //         ("mavros/global_position/global", 10, &FwControl::getGlobalLLA, this);
    // vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);
    // localPos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
    // takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
    //         ("mavros/cmd/takeoff");
    // att_thr_pub = nh.advertise<mavros_msgs::AttitudeTarget>
	// 		("mavros/setpoint_raw/attitude", 10);
    std::ostringstream ss;
    ss << "uav" << uavID << "/mavros/state";
    state_sub = nh.subscribe<mavros_msgs::State>
            (ss.str(), 10, &FwControl::getCurrentState, this);
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/global_position/global";
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            (ss.str(), 10, &FwControl::getGlobalLLA, this);
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/setpoint_velocity/cmd_vel";
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            (ss.str(), 10);
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/setpoint_position/local";
    localPos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (ss.str(), 10);
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/cmd/arming";
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (ss.str());
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/cmd/arming";
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (ss.str());
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/cmd/takeoff";
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            (ss.str());
    ss.str("");
    
    ss << "uav" << uavID << "/mavros/setpoint_raw/attitude";        
    att_thr_pub = nh.advertise<mavros_msgs::AttitudeTarget>
			(ss.str(), 10);

}

FwControl::~FwControl()
{

}

void FwControl::getCurrentState(const mavros_msgs::State::ConstPtr& state)
{
    current_state = *state;
}

void FwControl::getGlobalLLA(const sensor_msgs::NavSatFix::ConstPtr& lla)
{
    current_lla = *lla;
}

bool FwControl::CheckFCUConnection()
{
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

bool FwControl::Initialize()
{
    cmd_vel.twist.linear.x = 0.01;
    for (int i = 100; ros::ok() && i > 0; i--){
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

bool FwControl::Initialize2()
{
    cmd_pos.pose.position.x = 10.0;
    cmd_pos.pose.position.y = 0.0;
    cmd_pos.pose.position.z = 30.0;
    
    for (int i = 20; ros::ok() && i > 0; i--){
        localPos_pub.publish(cmd_pos);
        // std::cout << "pub cmd_pos" << std::endl;
        //ros::spinOnce();
        rate.sleep();
    }

    return true;
}

void FwControl::SwitchMode()
{
    offb_set_mode.request.custom_mode = controlMode;
    cmd_arm.request.value = true;
    last_request = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != controlMode &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent ){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(cmd_arm) &&
                    cmd_arm.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::Takeoff()
{
    cmd_takeoff.request.latitude = current_lla.latitude;
    cmd_takeoff.request.longitude = current_lla.longitude + 0.008;
    cmd_takeoff.request.altitude = current_lla.altitude + 100;
    cmd_vel.twist.linear.x = 10.;
    cmd_vel.twist.linear.y = 0.;
    cmd_vel.twist.linear.z = 0.;
    
    while(ros::ok()){
        if(!isTakeoff){
            if( current_state.mode == controlMode && current_state.armed){
            takeoff_client.call(cmd_takeoff);
            ROS_INFO("Takeoff");
            isTakeoff = cmd_takeoff.response.success;
            }
        }

        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::SwitchMode2()
{
    // send a few setpoints before starting
    cmd_pos.pose.position.x = 0.;
    cmd_pos.pose.position.y = 0.;
    cmd_pos.pose.position.z = 1.5;
    for (int i = 10; ros::ok() && i > 0; --i) {
        localPos_pub.publish(cmd_pos);
        ros::spinOnce();
        rate.sleep();
    }
    
    offb_set_mode.request.custom_mode = controlMode;
    cmd_arm.request.value = true;

    cmd_takeoff.request.latitude = current_lla.latitude;
    cmd_takeoff.request.longitude = current_lla.longitude + 0.004;
    cmd_takeoff.request.altitude = current_lla.altitude + 100.0;

    last_request = ros::Time::now();
    while(ros::ok()){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(cmd_arm) &&
                cmd_arm.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else if( !isTakeoff && current_state.armed){
                takeoff_client.call(cmd_takeoff);
                ROS_INFO("Takeoff");
                isTakeoff = cmd_takeoff.response.success;
        } else {
            if( current_state.mode != "OFFBOARD" && isTakeoff &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }
        
        if(current_state.armed && isTakeoff && current_state.mode == "OFFBOARD"){
            break;
        }

        localPos_pub.publish(cmd_pos);
        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::TrackCircle(std::vector<float> ctr, float radius, float freq)
{
    float period = 200; //second
    float step = 0.01;
    float omega = 2*M_PI*freq;
    geometry_msgs::Point waypoint;
    geometry_msgs::Point errPos;

    for(float t = 0; t <= period;){

        //std::cout << "center x: " << ctr[0] << std::endl;
        waypoint.x = ctr[0] + radius*sin(omega*t);
        waypoint.y = ctr[1] + radius*cos(omega*t);
        waypoint.z = ctr[2];

        cmd_pos.pose.position.x = waypoint.x;
        cmd_pos.pose.position.y = waypoint.y;
        cmd_pos.pose.position.z = waypoint.z;

        std::cout << "Pass through the local location: " << cmd_pos.pose.position.x \
            << ", " << cmd_pos.pose.position.y << ", " << cmd_pos.pose.position.z \
            << std::endl;

        localPos_pub.publish(cmd_pos);
        rate.sleep();
        
        t += step;
    }
}

void FwControl::TrackCircle2(std::vector<float> ctr, float radius, float freq)
{
    float period = 200; //second
    float step = 0.01;
    float omega = 2*M_PI*freq;
    geometry_msgs::Point waypoint;
    geometry_msgs::Point errPos;

    std::vector<std::vector<float>> guidePath ({{50, 0}, {55, 10}, {60, 20}, {60, 30}, {20, 20}});

    for(int i = 0; i < 5; i++){
        cmd_pos.pose.position.x = guidePath[i][0];
        cmd_pos.pose.position.y = guidePath[i][0];
        cmd_pos.pose.position.z = ctr[2];

        std::cout << "Guide the location: " << cmd_pos.pose.position.x \
            << ", " << cmd_pos.pose.position.y << ", " << cmd_pos.pose.position.z \
            << std::endl;
    }


    for(float t = 0; t <= period;){
        waypoint.x = ctr[0] + radius*sin(omega*t);
        waypoint.y = ctr[1] + radius*cos(omega*t);
        waypoint.z = ctr[2];

        cmd_pos.pose.position.x = waypoint.x;
        cmd_pos.pose.position.y = waypoint.y;
        cmd_pos.pose.position.z = waypoint.z;

        std::cout << "Pass through the local location: " << cmd_pos.pose.position.x \
            << ", " << cmd_pos.pose.position.y << ", " << cmd_pos.pose.position.z \
            << std::endl;

        localPos_pub.publish(cmd_pos);
        rate.sleep();
        
        t += step;
    }
}

void FwControl::Vel_cmd(double vx, double vy, double vz)
{
    cmd_vel.twist.linear.x = vx;
    cmd_vel.twist.linear.y = vy;
    cmd_vel.twist.linear.z = vz;

    vel_pub.publish(cmd_vel);
    std::cout << "The vel cmd is (" << cmd_vel.twist.linear.x << ", " << cmd_vel.twist.linear.y << ", " << cmd_vel.twist.linear.z << ")" << std::endl;

    ros::spinOnce();
    rate.sleep();
    
}

void FwControl::Att_Thr_cmd(double rate_x, double rate_y, double rate_z, double thr_cmd)
{
    uavAttitudeThrustCmd.type_mask = 132;
    uavAttitudeThrustCmd.orientation.x = 0.;
    uavAttitudeThrustCmd.orientation.y = 0.;
    uavAttitudeThrustCmd.orientation.z = 0.;
    uavAttitudeThrustCmd.orientation.w = 0.;
    uavAttitudeThrustCmd.body_rate.x = rate_x;
    uavAttitudeThrustCmd.body_rate.y = rate_y;
    uavAttitudeThrustCmd.body_rate.z = rate_z;
    uavAttitudeThrustCmd.thrust = thr_cmd;

    att_thr_pub.publish(uavAttitudeThrustCmd);
    std::cout << "The rate_cmd is (" << uavAttitudeThrustCmd.body_rate.x << ", " << uavAttitudeThrustCmd.body_rate.y << ", " << uavAttitudeThrustCmd.body_rate.z << ")" << std::endl;
    std::cout << "The thrust cmd is " << uavAttitudeThrustCmd.thrust << std::endl;

    ros::spinOnce();
    rate.sleep();
}