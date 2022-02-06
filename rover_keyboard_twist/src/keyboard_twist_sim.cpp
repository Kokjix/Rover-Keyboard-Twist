#include "definitions.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Keyboard_Teleop",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    std::signal(SIGINT, signal_handler);
    while (ros::ok)
    {
        ros::spinOnce();
        KeyboardTwist keyboard(&nh);
    }
    
    return 0;
}
