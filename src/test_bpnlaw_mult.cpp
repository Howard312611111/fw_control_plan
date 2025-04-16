//////////
// main 
//////////

#include <iostream>

#include <cassert>
#include <stdlib.h>
#include <UAVState.h>

using namespace std;

// test the BPNGlaw_3D function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bpnlaw");
    UAVState uav;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
    
    double xx4 = -100., yy4 = -100., zz4 = 170., dis_wayp4;
    double ang_azi, ang_ele;
    int s = 1;

    // initial condition
    cout << "The initial point is (" << xx4 << ", " << yy4 << ", " << zz4 << ")" << endl;
    dis_wayp4 = uav.get_dis_waypoint(xx4, yy4, zz4);
    cout << "The distance between UAV and initial point is " << dis_wayp4 << " m" << endl;
    uav.showPosVel();
    while(ros::ok()) {
        for (int i = 0; i < 100; i++) {
            uav.vel_cmd(1., 0., 0.);
            rate.sleep();
            uav.showPosVel();
            ros::spinOnce();
        }
        break;
        // uav.vel_cmd(1., 0., 0.);
        // rate.sleep();
        // uav.showPosVel();
        // dis_wayp4 = uav.get_dis_waypoint(xx4, yy4, zz4);
        // cout << "The distance between UAV and initial point is " << dis_wayp4 << "m\n" << endl;

        // if (dis_wayp4 < 25.){
        //     break;
        // }
        // ros::spinOnce();
    }

    // BPNG part
    while(ros::ok()) {
        uav.showPosVel();
        //uav.test_raw_attitude(2., 2., 0.);
        uav.bpng_acc_cmd(-45, -30);
        uav.bpng_acc_cmd(0, -30);
        uav.bpng_acc_cmd(45, -30);

        rate.sleep();
        ros::spinOnce();
    }

    return(0);
}








