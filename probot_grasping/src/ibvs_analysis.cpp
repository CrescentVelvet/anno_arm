#include <opencv2/opencv.hpp>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;

#define NUM 4
vpColVector v;
vpFeaturePoint ppp[NUM];
// bool iscircle(double rough_radius, Mat image, Point center)
// {
//     //多扇区圆检测
//     double eta = 30;
//     int tao = 8;
//     int r[2] = {0.8 * rough_radius, 1.2 * rough_radius};
//     double R[8] = {0};
//     double alpha;
//     int s;
//     for (int delta_x = -r[1]; delta_x <= r[1]; delta_x++)
//     {
//         for (int delta_y = -r[1]; delta_y <= r[1]; delta_y++)
//         {
//             if (delta_x * delta_x + delta_y * delta_y > r[1] || delta_x * delta_x + delta_y * delta_y > r[1] < r[0])
//                 continue;
//             alpha = atan2(delta_y, delta_x);
//             R[int(floor((alpha + M_PI) / (M_PI / 4)))] += image.at<uchar>(center.x + delta_x, center.y + delta_y) / 255;
//         }
//     }
//     for (int i = 0; i < 8; i++)
//     {
//         if (R[i] > eta)
//             s++;
//     }
//     return s >= tao;
// }

// int corner_num_in_circle(Point center, double radius, std::vector<cv::Point2f> &corners)
// {
//     int count = 0;
//     for (int i = 0; i < corners.size(); i++)
//     {
//         if ((corners[i].x - center.x) * (corners[i].x - center.x) + (corners[i].y - center.y) * (corners[i].y - center.y) <= radius * radius)
//         {
//             count++;
//         }
//     }
//     return count;
// }

void myCallback(const std_msgs::Float64MultiArray &msg)
{
    // detect_square(image_color);
    // cv::Mat image_gray;
    // Mat edges;
    // //边缘检测
    // int g_nThresh = 30; //当前阀值
    // int g_nMaxThresh = 175;
    // Mat image_color_blurred;
    // cv::Mat scharr_img, scharr_imgx, scharr_imgy, absscharr_imgx, absscharr_imgy;
    // cv::GaussianBlur(image_color, image_color_blurred, cv::Size(9, 9), 0, 0, cv::BORDER_DEFAULT);
    // cv::cvtColor(image_color_blurred, image_gray, CV_BGR2GRAY);
    // threshold(image_gray, image_gray, 90, 255, THRESH_BINARY); //图像二值化，，注意阈值变化
    // cv::Scharr(image_color_blurred, scharr_imgx, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    // cv::convertScaleAbs(scharr_imgx, absscharr_imgx);
    // cv::Scharr(image_color_blurred, scharr_imgy, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    // cv::convertScaleAbs(scharr_imgy, absscharr_imgy);
    // cv::addWeighted(absscharr_imgx, 1, absscharr_imgy, 1, 0, scharr_img);
    // // cv::imshow("image_color", image_color);
    // cv::imshow("image_color_blurred", image_color_blurred);
    // cv::imshow("image_gray", image_gray);
    // cv::imshow("scharr_img", scharr_img);
    // //高斯滤波
    // cv::cvtColor(scharr_img, edges, CV_BGR2GRAY);
    // Mat orig_edges = edges.clone();
    // GaussianBlur(edges, edges, Size(3, 3), 2, 2);
    // edges = 255 - edges - 20;
    // imshow("edges", edges);
    // vector<Vec3f> circles;
    // //霍夫圆
    // Mat image_color_hough = image_color.clone();
    // HoughCircles(edges, circles, CV_HOUGH_GRADIENT, 1, 200, 100, 20, 0, 50);
    // for (size_t i = 0; i < circles.size(); i++)
    // {
    //     Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //     int radius = cvRound(circles[i][2]);
    //     //绘制圆心
    //     circle(image_color_hough, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    //     //绘制圆轮廓
    //     circle(image_color_hough, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    // }
    // imshow("circle extracted", image_color_hough);
    // //多扇区圆检测
    // vector<Vec3f> certain_circles;
    // Mat image_color_certain_hough = image_color.clone();
    // for (size_t i = 0; i < circles.size(); i++)
    // {
    //     Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //     int radius = cvRound(circles[i][2]);
    //     if (iscircle(radius, edges, center))
    //         certain_circles.push_back(circles[i]);
    // }
    // for (size_t i = 0; i < certain_circles.size(); i++)
    // {
    //     Point center(cvRound(certain_circles[i][0]), cvRound(certain_circles[i][1]));
    //     int radius = cvRound(certain_circles[i][2]);
    //     //绘制圆心
    //     circle(image_color_certain_hough, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    //     //绘制圆轮廓
    //     circle(image_color_certain_hough, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    // }
    // imshow("image_color_certain_hough", image_color_certain_hough);
    // //设置角点检测参数
    // std::vector<cv::Point2f> corners;
    // int max_corners = 1000;
    // double quality_level = 0.04;
    // double min_distance = 3.0;
    // int block_size = 3;
    // bool use_harris = false;
    // double k = 0.04;
    // Mat image_color_corner = image_color.clone();
    // //角点检测
    // cv::goodFeaturesToTrack(image_gray,
    //                         corners,
    //                         max_corners,
    //                         quality_level,
    //                         min_distance,
    //                         cv::Mat(),
    //                         block_size,
    //                         use_harris,
    //                         k);
    // //将检测到的角点绘制到原图上
    // for (int i = 0; i < corners.size(); i++)
    // {
    //     cv::circle(image_color_corner, corners[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    // }
    // cv::imshow("house corner", image_color_corner);
    // //过滤霍夫圆
    // vector<Vec3f> filtered_circles;
    // for (size_t i = 0; i < circles.size(); i++)
    // {
    //     Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //     int radius = cvRound(circles[i][2]);
    //     if (corner_num_in_circle(center, 2 * radius, corners) - corner_num_in_circle(center, 1.4 * radius, corners) < 1 && corner_num_in_circle(center, 1.5 * radius, corners) > 3)
    //     {
    //         filtered_circles.push_back(circles[i]);
    //     }
    // }
    // cout << "--------------------------------" << filtered_circles.size() << endl;
    // //用filtered霍夫圆筛选角点
    // std::vector<cv::Point2f> filtered_corners;
    // if (filtered_circles.size() == 1)
    // {
    //     Point center(cvRound(filtered_circles[0][0]), cvRound(filtered_circles[0][1]));
    //     int radius = cvRound(filtered_circles[0][2]);
    //     //绘制圆心
    //     circle(image_color, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    //     //绘制圆轮廓
    //     circle(image_color, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    //     for (size_t j = 0; j < corners.size(); j++)
    //     {
    //         if ((corners[j].x - center.x) * (corners[0].x - center.x) + (corners[j].y - center.y) * (corners[j].y - center.y) <= 1.3 * 1.3 * radius * radius)
    //         {
    //             filtered_corners.push_back(corners[j]);
    //         }
    //     }
    //     for (int i = 0; i < corners.size(); i++)
    //     {
    //         cv::circle(image_color, filtered_corners[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    //     }
    // }
    // cout << "--------------------------------" << filtered_corners.size() << endl;
    // imshow("final", image_color);

    // for (int i = 0; i < 2 * NUM; i += 2)
    // {
    //     ppp[i/2].set_x(msg.data[i]);
    //     ppp[i/2].set_y(msg.data[i + 1]);
    //     ppp[i/2].set_Z(0);
    //     // std::cout << "123" << std::endl;
    // }
    // cv::waitKey(8);
}

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo,
                        const vpCameraParameters &cam)
{
    vpImagePoint cog;
    for (unsigned int i = 0; i < NUM; i++)
    {
        point[i].project(cMo);
        vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibvs_analysis");
    ros::NodeHandle n;
    // ros::Subscriber my_subscriber = n.subscribe("/display_img", 10, myCallback);
    ros::Publisher my_publisher = n.advertise<std_msgs::Float64MultiArray>("/cmd_vel", 100);
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < 6; i++)
    {
        msg.data.push_back(0);
    }
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));
    std::vector<vpPoint> point;
    point.push_back(vpPoint(0.1, -0.1, 0));
    point.push_back(vpPoint(0.1, 0.1, 0));
    point.push_back(vpPoint(-0.1, 0.1, 0));
    point.push_back(vpPoint(-0.1, -0.1, 0));
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    vpFeaturePoint pd[NUM];
    for (unsigned int i = 0; i < NUM; i++)
    {
        point[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], point[i]);
        point[i].track(cMo);
        vpFeatureBuilder::create(ppp[i], point[i]);
        task.addFeature(ppp[i], pd[i]);
    }
    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.getPosition(wMc);
    wMo = wMc * cMo;
    vpImage<unsigned char> Iint(480, 640, 255);
    vpImage<unsigned char> Iext(480, 640, 255);
    vpDisplayX displayInt(Iint, 0, 0, "Internal view");
    vpCameraParameters cam(800, 800, 400, 400);
    while (ros::ok())
    {
        ros::spinOnce();

        robot.getPosition(wMc);
        cMo = wMc.inverse() * wMo;
        for (unsigned int i = 0; i < NUM; i++)
        {
            point[i].track(cMo);
            vpFeatureBuilder::create(ppp[i], point[i]);
        }
        v = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        for (int i = 0; i < 6; i++)
        {
            msg.data.at(i) = v.data[i];
            // msg.data.at(i) = 0;
        }
        my_publisher.publish(msg);
        vpDisplay::display(Iint);
        display_trajectory(Iint, point, cMo, cam);
        if (vpDisplay::getClick(Iint, false) || vpDisplay::getClick(Iext, false))
            break;
    }
    task.kill();
}
