#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "jacobi.hpp"

ros::Time t;
double begin;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        for (int i = 0; i < 6; i++)
        {
            init_pos.data.push_back(0);
        }
        pub_ = n_.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
        sub_status = n_.subscribe("/gazebo/link_states", 100, &SubscribeAndPublish::poseCallBack, this);
        sub_cmdvel = n_.subscribe("/cmd_vel", 100, &SubscribeAndPublish::cmdvelCallback, this);
    }

    void cmdvelCallback(const std_msgs::Float64MultiArray &msg)
    {
        this->cmdvx = msg.data[0];
        this->cmdvy = msg.data[1];
        this->cmdvz = msg.data[2];
        this->cmdwx = msg.data[3];
        this->cmdwy = msg.data[4];
        this->cmdwz = msg.data[5];
    }
    void calLinkVel()
    {
        ros::Time now = ros::Time::now();
        double duration = (now.toSec() - t.toSec()) - begin;
        Eigen::Matrix<double, 6, 1> vel;
        // if (duration <= 5)
        // {
            // vel << vx, vy, vz, wx, wy, wz;
            vel << this->cmdvx, this->cmdvy, this->cmdvz, this->cmdwx, this->cmdwy, this->cmdwz;
            // vel << 0.005, 0.0, 0.0, 0.0, 0.0, 0.0;
            // std::cout << ros::Time().now() << std::endl;
        // }

        // else
            // vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        thetaVel = jacobi.inverse() * vel;
    }

    void poseCallBack(const gazebo_msgs::LinkStatesConstPtr &msg)
    {
        for (int i = 2; i < 8; i++)
        {
            pos[i - 2].x = msg->pose[i].position.x;
            pos[i - 2].y = msg->pose[i].position.y;
            pos[i - 2].z = msg->pose[i].position.z;
        }
        Eigen::Vector3d Z_4, Z_6;
        double theta = atan((pos[4].z - pos[3].z) / (pos[4].x - pos[3].x));
        Z_4 << cos(theta), 0, sin(theta);
        double alpha = atan((pos[5].z - pos[4].z) / (pos[5].x - pos[4].x));
        Z_6 << cos(alpha), 0, sin(alpha);
        jacobi = calJacobi(pos, Z_4, Z_6);
        calLinkVel();
        for (int i = 0; i < 6; i++)
        {
            init_pos.data.at(i) = thetaVel(i, 0);
        }
        pub_.publish(init_pos);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_status;
    ros::Subscriber sub_cmdvel;
    Eigen::Matrix<double, 6, 6> jacobi;
    Eigen::Matrix<double, 6, 1> thetaVel;
    geometry_msgs::Point pos[6];
    double currentVel = 0;
    double cmdvx, cmdvy, cmdvz, cmdwx, cmdwy, cmdwz;
    std_msgs::Float64MultiArray init_pos;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_subscriber");

    ros::NodeHandle nh;
    t = ros::Time(0);

    SubscribeAndPublish SAPObject;
    do
    {
        ros::Time now_1 = ros::Time::now();
        begin = now_1.toSec() - t.toSec();
    } while (begin == 0);

    ros::spin();

    return 0;
}