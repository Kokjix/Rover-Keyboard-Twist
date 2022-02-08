#include <iostream>
#include <csignal>
#include "ros/ros.h"
#include "definitions.hpp"

/**
@author: Baran Berk Bağcı
@date: 7.02.2022
*/

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Keyboard_Teleop",ros::init_options::NoSigintHandler); //init node
    ros::NodeHandle nh; // declare node handle

    std::signal(SIGINT, signal_handler); // signal handler
    while (ros::ok) // while loop
    {
        ros::spinOnce();
        KeyboardTwist keyboard(&nh); // main class
    }
    
    return 0;
}
