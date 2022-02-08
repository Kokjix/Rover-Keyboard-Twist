#include <iostream>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h> //STDIN_FILENO
#include <csignal>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
@author: Baran Berk Bağcı
@date: 7.02.2022
*/

namespace
{
volatile std::sig_atomic_t gSignalStatus;
} // namespace for unbuffered char input


void signal_handler(int signal)
{
    gSignalStatus = signal;
    std::cout <<"\n\n\n GOODBYE :(( \n\n\n"; // stream GOODBYE :((
    exit(0); // successful program termination
}


class KeyboardTwist // keyboard twist class
{
private:
    ros::Publisher twist_pub; // twist publisher
    std::string cmd_topic; // cmd topic get argument from launch file

public:
    KeyboardTwist(ros::NodeHandle *nh) // class constructor
    {
        nh->getParam("/keyboard_twist_sim/topic",cmd_topic); // node handler get topic name from launch file argument
        twist_pub = nh->advertise<geometry_msgs::Twist>(cmd_topic, 100); // twist publisher
        std::signal(SIGINT, signal_handler); // signal handler
        while (ros::ok) // while loop
        {
            ros::spinOnce();
            run_keyboard(); // main method
        }
        
    };
    
    int getch() // unbuffered getchar function
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

    void run_keyboard() // main method
    {
        char key_input; // key value
        key_input = getch(); // get key value from getch function
        geometry_msgs::Twist twist; // declared twist
        switch (key_input) // switch-case for twist object
        {
            case 'w': case 'W': // w key pressed + linear x value 
                twist.linear.x = 0.5;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 's': case 'S': // s key pressed - linear x value
                twist.linear.x = -0.5;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 'a': case 'A': // a key pressed + angular value
                twist.angular.z = 0.1;
                std::cout << "\x1B[2J\x1B[H" << std::endl;  // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 'd': case 'D': // d key pressed - angular value
                twist.angular.z = -0.1;
                std::cout << "\x1B[2J\x1B[H" << std::endl;  // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 'q': case 'Q': // q key pressed + linear y value
                twist.linear.y = 0.5;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 'e': case 'E': // e key pressed - linear y value
                twist.linear.y = -0.5;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                break;

            case 't': case 'T': // t key pressed publish all twist alues 0 and then terminate script
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist
                exit(0); // successful program termination

            default: // when other q, w, e, a, s, d or t than keys pressed program publish all 0 twist 
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                std::cout << "\x1B[2J\x1B[H" << std::endl; // clear terminal
                ROS_INFO_STREAM(twist); // ros info but stream
                twist_pub.publish(twist); // publish twist 
                break;
            }

    }
    
    
    ~KeyboardTwist() // destructor
    {

    };

};

// KeyboardTwist::KeyboardTwist(/* args */)
// {
// }

// KeyboardTwist::~KeyboardTwist()
// {
// }
