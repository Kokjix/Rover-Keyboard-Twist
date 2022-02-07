#include <iostream>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h> //STDIN_FILENO
#include <csignal>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/*
@author: Baran Berk Bağcı
@date: 7.02.2022
*/

namespace
{
volatile std::sig_atomic_t gSignalStatus;
}


void signal_handler(int signal)
{
    gSignalStatus = signal;
    std::cout <<"\n\n\n GOODBYE :(( \n\n\n";
    exit(0);
}


class KeyboardTwist
{
private:
    ros::Publisher twist_pub;
    std::string cmd_topic;

public:
    KeyboardTwist(ros::NodeHandle *nh)
    {
        nh->getParam("/keyboard_twist_sim/topic",cmd_topic);
        twist_pub = nh->advertise<geometry_msgs::Twist>(cmd_topic, 100);
        std::signal(SIGINT, signal_handler);
        while (ros::ok)
        {
            ros::spinOnce();
            run_keyboard();
        }
        
    };
    
    int getch()
    {
        static struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);  // save old settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON);                // disable buffering
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings
        int c = getchar();  // read character (non-blocking)

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
        return c;
    }

    void run_keyboard()
    {
        char key_input;
        key_input = getch();
        geometry_msgs::Twist twist;
        ros::Rate rate(150);
        switch (key_input)
        {
            case 'w': case 'W':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.linear.x = 10;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 's': case 'S':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.linear.x = -10;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 'a': case 'A':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.angular.z = 2*3.1415926535;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 'd': case 'D':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.angular.z = -2*3.1415926535;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 'q': case 'Q':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.linear.y = 10;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 'e': case 'E':
                // twist.linear.x = 0;
                // twist.linear.y = 0;
                // twist.angular.z = 0;
                twist.linear.y = -10;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                twist_pub.publish(twist);
                break;

            case 't': case 'T':
                exit(0);

            default:
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                std::cout << "\x1B[2J\x1B[H" << std::endl;
                ROS_INFO_STREAM(twist);
                break;
            }

            // key_input = 'j';
            // std::cout << key_input << std::endl;
            //twist_pub.publish(twist);
            rate.sleep();
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            twist_pub.publish(twist);
    }
    
    
    ~KeyboardTwist()
    {

    };

};

// KeyboardTwist::KeyboardTwist(/* args */)
// {
// }

// KeyboardTwist::~KeyboardTwist()
// {
// }
