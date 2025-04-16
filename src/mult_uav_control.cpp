#include "mult_FwControl.hpp"

#define LOITER 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control");
    FwControl fw0(0), fw1(1), fw2(2);

#if LOITER
    bool state;
    std::vector<float> targetCtr ({0, 0, 150}); 

    state = fw0.CheckFCUConnection();
    std::cout << "FCU Connection State: " << state << std::endl;
    state = fw0.Initialize2();
    std::cout << "Finished Initializing: " << state << std::endl;
    fw0.SwitchMode2();
    std::cout << "Velocity command in OFFBOARD mode" << std::endl;

    state = fw1.CheckFCUConnection();
    std::cout << "FCU Connection State: " << state << std::endl;
    state = fw1.Initialize2();
    std::cout << "Finished Initializing: " << state << std::endl;
    fw1.SwitchMode2();
    std::cout << "Velocity command in OFFBOARD mode" << std::endl;

    state = fw2.CheckFCUConnection();
    std::cout << "FCU Connection State: " << state << std::endl;
    state = fw2.Initialize2();
    std::cout << "Finished Initializing: " << state << std::endl;
    fw2.SwitchMode2();
    std::cout << "Velocity command in OFFBOARD mode" << std::endl;

    for(int i = 100; ros::ok() && i > 0; --i){
        fw0.Vel_cmd(1., 0., 0.);
        fw1.Vel_cmd(1., 0., 0.);
        fw2.Vel_cmd(1., 0., 0.);
    }
    
    // for(int i = 200; ros::ok() && i > 0; --i){
    //     fw.Att_Thr_cmd(0., 0.3, 0., 0.2);
    // }
    // for(int i = 200; ros::ok() && i > 0; --i){
    //     fw.Att_Thr_cmd(0., -0.5, 0., 0.2);
    // }
    // for(int i = 50; ros::ok() && i > 0; --i){
    //     fw.Att_Thr_cmd(0.5, 0.3, 0., 0.5);
    // }
    // for(int i = 50; ros::ok() && i > 0; --i){
    //     fw.Att_Thr_cmd(-0.5, -0.3, 0., 0.5);
    // }

    
    // std::cout << "Start Loitering" << std::endl;
    // fw.TrackCircle2(targetCtr, 60, 0.25/(2*M_PI));
#endif    

    return 0;
}