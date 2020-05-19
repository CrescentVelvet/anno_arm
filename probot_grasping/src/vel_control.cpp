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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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
        pub_ = n_.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 1);
        sub_status = n_.subscribe("/gazebo/link_states", 1, &SubscribeAndPublish::poseCallBack, this);
        sub_cmdvel = n_.subscribe("/cmd_vel", 1, &SubscribeAndPublish::cmdvelCallback, this);
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
        // if (duration <= 80)
        // {
            // vel << vx, vy, vz, wx, wy, wz;
            vel << this->cmdvx, this->cmdvy, this->cmdvz, this->cmdwx, this->cmdwy, this->cmdwz;
            // z方向反向
            // vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            // std::cout << ros::Time().now() << std::endl;
        // }

        // else
            // vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // thetaVel = jacobi.inverse() * vel;
        thetaVel = jacobian.inverse() * vel;
    }

    void poseCallBack(const gazebo_msgs::LinkStatesConstPtr &msg)
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0315);

        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position, jacobian);
        // ROS_INFO_STREAM("Jacobian: \n"
                        // << jacobian << "\n");
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
        // ROS_INFO_STREAM("Jacobian: \n"
        // << jacobi << "\n");
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
    Eigen::MatrixXd jacobian;
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