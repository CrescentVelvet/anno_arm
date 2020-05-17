#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testvel");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/cmd_vel", 100);
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < 6; i++)
    {
        msg.data.push_back(0);
    }
    msg.data.at(0) = 0.005;
    msg.data.at(1) = 0.000;
    msg.data.at(2) = 0.000;
    while (ros::ok())
    {
        vel_pub.publish(msg);
    }

    return 0;
}
