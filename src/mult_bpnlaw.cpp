//////////
// main 
//////////

#include <iostream>

#include <cassert>
#include <stdlib.h>
#include <UAVState_mult.h>

using namespace std;

// test the BPNGlaw_3D function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bpnlaw");
    // UAVState uav0, uav1, uav2;
    UAVState uavs;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);
    
    double xx0 = -100., yy0 = 50.,  zz0 = 170., dis_wayp0;
    double xx1 = -100., yy1 = 0.,   zz1 = 170., dis_wayp1;
    double xx2 = -100., yy2 = -50., zz2 = 170., dis_wayp2;
    // double ang_azi, ang_ele;
    // int s = 1;
    int flag0 = 0;
    int flag1 = 0;
    int flag2 = 0;

    // switching condition
    cout << "The switch point for uav0 is (" << xx0 << ", " << yy0 << ", " << zz0 << ")" << endl;
    dis_wayp0 = uavs.get_dis_waypoint0(xx0, yy0, zz0);
    cout << "The distance between uav0 and switch point is " << dis_wayp0 << " m" << endl;

    cout << "The switch point for uav1 is (" << xx1 << ", " << yy1 << ", " << zz1 << ")" << endl;
    dis_wayp1 = uavs.get_dis_waypoint1(xx1, yy1, zz1);
    cout << "The distance between uav1 and switch point is " << dis_wayp1 << " m" << endl;

    cout << "The switch point for uav2 is (" << xx2 << ", " << yy2 << ", " << zz2 << ")" << endl;
    dis_wayp2 = uavs.get_dis_waypoint2(xx2, yy2, zz2);
    cout << "The distance between uav2 and switch point is " << dis_wayp2 << " m" << endl;

    // uav.showPosVel();
    while(ros::ok()) {
        // for (int i = 0; i < 20; i++) {
        //     // uav0.vel_cmd0(1., 0., 0.);
        //     // uav1.vel_cmd1(1., 0., 0.);
        //     // uav2.vel_cmd2(1., 0., 0.);
        //     uavs.vel_cmd0(1., 0., 0.);
        //     uavs.vel_cmd1(1., 0., 0.);
        //     uavs.vel_cmd2(1., 0., 0.);

        //     rate.sleep();
        //     // uav.showPosVel();
        //     ros::spinOnce();
        // }
        // break;
        // uav.vel_cmd(1., 0., 0.);
        // rate.sleep();
        // uavs.showPosVel0();
        // uavs.showPosVel1();
        // uavs.showPosVel2();
        std::cout << "\033[1;1H\033[2J"; // 清除螢幕並將光標移動到第一行

        
        if (flag0==0 && flag1==0 && flag2==0) {
            uavs.vel_cmd0(1., 0., 0.);
            uavs.vel_cmd1(1., 0., 0.);
            uavs.vel_cmd2(1., 0., 0.);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==1 && flag1==0 && flag2==0) {
            uavs.bpng_acc_cmd0(45, -30);
            uavs.vel_cmd1(1., 0., 0.);
            uavs.vel_cmd2(1., 0., 0.);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==0 && flag1==1 && flag2==0) {
            uavs.vel_cmd0(1., 0., 0.);
            uavs.bpng_acc_cmd1(0, -30);
            uavs.vel_cmd2(1., 0., 0.);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==0 && flag1==0 && flag2==1) {
            uavs.vel_cmd0(1., 0., 0.);
            uavs.vel_cmd1(1., 0., 0.);
            uavs.bpng_acc_cmd2(-45, -30);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==1 && flag1==1 && flag2==0) {
            uavs.bpng_acc_cmd0(45, -30);
            uavs.bpng_acc_cmd1(0, -30);
            uavs.vel_cmd2(1., 0., 0.);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==0 && flag1==1 && flag2==1) {
            uavs.vel_cmd0(1., 0., 0.);
            uavs.bpng_acc_cmd1(0, -30);
            uavs.bpng_acc_cmd2(-45, -30);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==1 && flag1==0 && flag2==1) {
            uavs.bpng_acc_cmd0(45, -30);
            uavs.vel_cmd1(1., 0., 0.);
            uavs.bpng_acc_cmd2(-45, -30);
            // rate.sleep();
            // ros::spinOnce();
        }
        if (flag0==1 && flag1==1 && flag2==1) {
            uavs.bpng_acc_cmd0(45, -30);
            uavs.bpng_acc_cmd1(0, -30);
            uavs.bpng_acc_cmd2(-45, -30);
            // rate.sleep();
            // ros::spinOnce();
        }
        dis_wayp0 = uavs.get_dis_waypoint0(xx0, yy0, zz0);
        cout << "The distance between UAV 0 and initial point is " << dis_wayp0 << "m\n" << endl;
        if (dis_wayp0 < 25.){
            flag0 = 1;
        }
        dis_wayp1 = uavs.get_dis_waypoint1(xx1, yy1, zz1);
        cout << "The distance between UAV 1 and initial point is " << dis_wayp1 << "m\n" << endl;
        if (dis_wayp1 < 25.){
            flag1 = 1;
        }
        dis_wayp2 = uavs.get_dis_waypoint2(xx2, yy2, zz2);
        cout << "The distance between UAV 2 and initial point is " << dis_wayp2 << "m\n" << endl;
        if (dis_wayp2 < 25.){
            flag2 = 1;
        }

        int c = 0;
        for(auto i_hit : uavs.is_hit){
            printf("[%d] is %s", c, i_hit ? "not yet hit":"hitted");
        }


        ros::spinOnce();
        rate.sleep();
    }

    // BPNG part
    while(ros::ok()) {
        // uav.showPosVel();
        // uav.test_raw_attitude(2., 2., 0.);
        // uav0.bpng_acc_cmd0(45, -30);
        // uav1.bpng_acc_cmd1(0, -30);
        // uav2.bpng_acc_cmd2(-45, -30);

        uavs.bpng_acc_cmd0(45, -30);
        uavs.bpng_acc_cmd1(0, -30);
        uavs.bpng_acc_cmd2(-45, -30);
        rate.sleep();
        ros::spinOnce();
    }

    return(0);
}








