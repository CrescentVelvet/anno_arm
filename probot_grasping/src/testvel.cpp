#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testvel");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/cmd_vel", 1);
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < 6; i++)
    {
        msg.data.push_back(0);
    }
    msg.data[0] = 0.0;
    msg.data[1] = 0.0;
    msg.data[2] = 0.0;
    msg.data[3] = 0.0;
    msg.data[4] = 0.0;
    msg.data[5] = 0.1;

    while (ros::ok())
    {
        // std::cout << "enter : ";
        // for (int i = 0; i < 6; i++)
        // {
            // std::cout << msg.data.at(0);
        // }
        vel_pub.publish(msg);
    }

    return 0;
}
