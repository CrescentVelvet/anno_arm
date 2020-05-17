#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

Eigen::Matrix<double, 6, 6> calJacobi(geometry_msgs::Point pos[], Eigen::Vector3d Z_4, Eigen::Vector3d Z_6){
    Eigen::Matrix<double, 3, 6> jacobiV ;
    Eigen::Matrix<double, 3, 6> jacobiW ;
    Eigen::Matrix<double, 6, 6> jacobi ;
    Eigen::Vector3d Z_1, Z_2, Z_3, Z_5;
    Z_1<< 0,0,1;
    Z_2<< 0,-1,0;
    Z_3<< 0,-1,0;
    Z_5<< 0,-1,0;
    Eigen::Vector3d P[6];
    for(int i = 0; i<6; i++){
        P[i] << pos[i].x, pos[i].y, pos[i].z;
    }
    Eigen::Vector3d temp;
    temp<< 0,0,0;
    jacobiV<< Z_1.cross(P[5]-P[0]), Z_2.cross(P[5]-P[1]),
            Z_3.cross(P[5]-P[2]), Z_4.cross(P[5]-P[3]), Z_5.cross(P[5]-P[4]), temp;
    jacobiW << Z_1, Z_2, Z_3, Z_4, Z_5, Z_6;
    jacobi.block<3,6>(0,0) = jacobiV;
    jacobi.block<3,6>(3,0) = jacobiW;
    // std::cout<< jacobiV <<std::endl<<std::endl;
    return jacobi;
}

Eigen::Matrix<double, 6, 6> myCalJacob(std::vector<double> link)
{
    Eigen::Matrix<double, 3, 6> jacobV;
    Eigen::Matrix<double, 3, 6> jacobW;
    Eigen::Matrix<double, 6, 6> jacob;
    

    Eigen::Matrix<double, 6, 1> theta;
    theta << link[0], link[1], link[2], link[3], link[4], link[5];
    Eigen::Matrix<double, 6, 1> init_theta;
    init_theta << 0, M_PI / 2, 0, 0, -M_PI / 2, 0;
    Eigen::Matrix<double, 6, 1> a, alpha, d;
    alpha << 0, M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2;
    a<< 0, 0, 0.225, 0 ,0 ,0;
    d << 0.284, 0, 0, 0.2289, 0, 0.055;
    theta = theta + init_theta;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    for(int i = 0; i<6 ;i++){
        Eigen::Matrix4d temp ;
        temp << cos(theta[i]), -sin(theta[i]), 0, a[i],
            sin(theta[i])*cos(alpha[i]), cos(theta[i])*cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i])*d[i],
            sin(theta[i])*sin(alpha[i]), cos(theta[i])*sin(alpha[i]), cos(alpha[i]), cos(alpha[i])*d[i],
            0, 0, 0, 1;
        T = T * temp;
        jacobV.col(i) = T.block<3, 1>(0,3);
        jacobW.col(i) = T.block<3,3>(0,0).col(2);
    }
    for(int i=0; i<6; i++){
        jacobV.col(i) = jacobW.col(i).cross((jacobV.col(5)-jacobV.col(i)));
    }
    jacob.block<3,6>(0,0) = jacobV;
    jacob.block<3,6>(3,0) = jacobW;
    return jacob;
}