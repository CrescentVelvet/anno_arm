#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include "std_msgs/Float64MultiArray.h"

#define P 0.0001
#define I 1.0
#define D 0.00001

ros::Publisher my_publisher;
std_msgs::Float64MultiArray cmd_vel;
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
void myCallback(const sensor_msgs::ImageConstPtr &img)
{
    int g_nThresh = 30;
    int g_nMaxThresh = 175;
    static int fuck_count;
    cv::Mat imgimg;
    cv::Mat hsv_img, mask_img, green_img;
    cv::Mat gray_img, scharr_img, scharr_imgx, scharr_imgy, absscharr_imgx, absscharr_imgy;
    std::vector<cv::Point> msg_img;
    // 绿色过滤
    std_msgs::Float64MultiArray msg;
    imgimg = cv_bridge::toCvShare(img, "bgr8")->image;
    green_img = cv::Mat::zeros(imgimg.size(), imgimg.type());
    cv::cvtColor(imgimg, hsv_img, CV_BGR2HSV);
    cv::inRange(hsv_img, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255), mask_img);
    for (int r = 0; r < imgimg.rows; r++)
    {
        for (int c = 0; c < imgimg.cols; c++)
        {
            if (mask_img.at<uchar>(r, c) == 255)
            {
                green_img.at<cv::Vec3b>(r, c)[0] = imgimg.at<cv::Vec3b>(r, c)[0];
                green_img.at<cv::Vec3b>(r, c)[1] = imgimg.at<cv::Vec3b>(r, c)[1];
                green_img.at<cv::Vec3b>(r, c)[2] = imgimg.at<cv::Vec3b>(r, c)[2];
            }
        }
    }
    // 轮廓绘制
    gray_img = cv::Mat::zeros(green_img.size(), CV_32FC1);
    cv::GaussianBlur(green_img, green_img, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(green_img, gray_img, CV_BGR2GRAY);
    cv::Scharr(green_img, scharr_imgx, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(scharr_imgx, absscharr_imgx);
    cv::Scharr(green_img, scharr_imgy, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(scharr_imgy, absscharr_imgy);
    cv::addWeighted(absscharr_imgx, 0.5, absscharr_imgy, 0.5, 0, scharr_img);
    // 边缘检测
    std::vector<std::vector<cv::Point>> squares;
    std::vector<cv::Point> std_squares;
    std::vector<cv::Point> p_squares;
    static std::vector<cv::Point> old_p_squares;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> approx;
    cv::Mat jiao_img0(imgimg.size(), CV_8U), jiao_img;
    int ch[] = {1, 0}, jiao_N = 5;
    cv::mixChannels(&green_img, 1, &jiao_img0, 1, ch, 1);
    for (int l = 0; l < jiao_N; l++)
    {
        if (l == 0)
        {
            cv::dilate(scharr_img, jiao_img, cv::Mat(), cv::Point(-1, -1));
        }
        else
        {
            jiao_img = jiao_img0 >= (l + 0.5) * 255 / jiao_N;
        }
    }
    cv::findContours(jiao_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 角点检测
    for (int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
        if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx)))
        {
            double maxCosine = 0;
            for (int j = 2; j < 5; j++)
            {
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            if (maxCosine < 0.3)
            {
                squares.push_back(approx);
            }
        }
    }
    if (squares.size() != 0)
    {
        // std::cout << squares[0].size() << "个角点" << std::endl;
        for (int i = 0; i < squares[0].size(); i++)
        {
            p_squares.push_back(squares[0][i]);
        }
    }
    else
    {
        // std::cout << 0 << "个角点" << std::endl;
        for (int i = 0; i < 4; i++)
        {
            p_squares.push_back(cv::Point(0, 0));
        }
    }
    for (int i = 0; i < p_squares.size(); i++)
    {
        old_p_squares.push_back(cv::Point(0, 0));
    }
    const cv::Point *p = &p_squares[0];
    int n = (int)p_squares.size();
    if (p->x > 3 && p->y > 3)
    {
        // std::cout << p->x << std::endl;
        cv::polylines(imgimg, &p, &n, 1, true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
    // 角点匹配
    std_squares.push_back(cv::Point(520, 280));
    std_squares.push_back(cv::Point(520, 520));
    std_squares.push_back(cv::Point(280, 520));
    std_squares.push_back(cv::Point(280, 280));
    const cv::Point *std_p = &std_squares[0];
    int std_n = (int)std_squares.size();
    if (std_p->x > 0 && std_p->y > 0)
    {
        cv::polylines(imgimg, &std_p, &std_n, 1, true, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }
    // 记忆p_mean
    if (p_squares[2].x < 1e-3 && p_squares[2].y < 1e-3)
    {
        fuck_count++;
        if (fuck_count > 50)
        {
            cmd_vel.data[0] = 0;
            cmd_vel.data[1] = 0;
            cmd_vel.data[2] = 0;
            cmd_vel.data[5] = 0;
            my_publisher.publish(cmd_vel);
            cv::waitKey(4);
            return;
        }

        for (int i = 0; i < p_squares.size(); i++)
        {
            p_squares[i].x = old_p_squares[i].x;
            p_squares[i].y = old_p_squares[i].y;
        }
    }
    else
    {
        fuck_count = 0;
        for (int i = 0; i < p_squares.size(); i++)
        {

            old_p_squares[i].x = p_squares[i].x;
            old_p_squares[i].y = p_squares[i].y;
        }
    }
    // 计算平移
    cv::Point p_mean, std_mean;
    std::vector<cv::Point>::iterator p_it, std_it;
    static double old_p_mean_x, old_p_mean_y;
    p_mean.x = 0;
    p_mean.y = 0;
    std_mean.x = 0;
    std_mean.y = 0;
    for (p_it = p_squares.begin(); p_it != p_squares.end(); p_it++)
    {
        p_mean.x += p_it->x;
        p_mean.y += p_it->y;
    }
    for (std_it = std_squares.begin(); std_it != std_squares.end(); std_it++)
    {
        std_mean.x += std_it->x;
        std_mean.y += std_it->y;
    }
    p_mean.x = p_mean.x / p_squares.size();
    p_mean.y = p_mean.y / p_squares.size();
    std_mean.x = std_mean.x / std_squares.size();
    std_mean.y = std_mean.y / std_squares.size();
    // std::cout<<p_mean<<std::endl;
    // 计算旋转
    double angle, angle_1, angle_2;
    double min_angle_p = 0;
    double min_angle_std = 0;
    double x1, y1, x2, y2;
    for (int i = 0; i < p_squares.size(); i++)
    {
        x1 = p_squares[i].x - p_mean.x;
        y1 = p_squares[i].y - p_mean.y;
        angle_1 = atan2l(y1, x1);
        if (angle_1 < min_angle_p)
        {
            min_angle_p = angle_1;
        }
        x2 = std_squares[i].x - std_mean.x;
        y2 = std_squares[i].y - std_mean.y;
        angle_2 = atan2l(y2, x2);
        if (angle_2 < min_angle_std)
        {
            min_angle_std = angle_2;
        }
    }

    angle = min_angle_std - min_angle_p;
    // if (angle > M_PI)
    // {
    //     angle -= 2 * M_PI;
    // }
    // else if (angle < -M_PI)
    // {
    //     angle += 2 * M_PI;
    // }
    // if (angle > M_PI / 2)
    // {
    //     angle -= M_PI / 2;
    // }
    // else if (angle < -M_PI / 2)
    // {
    //     angle += M_PI / 2;
    // }
    // std::cout << min_angle_p << std::endl;
    // std::cout << min_angle_std << std::endl;
    // std::cout << angle << std::endl;
    /// PID controller
    // X****************************
    double error_x = p_mean.y - std_mean.y;
    // std::cout << error_x << std::endl;
    // static double integra_x += error_x;
    static double last_error_x;
    double diff_x = error_x - last_error_x;
    last_error_x = error_x;
    double v_x = P * error_x + D * diff_x;
    // Y****************************
    double error_y = p_mean.x - std_mean.x;
    // static double integra_y += error_y;
    static double last_error_y;
    double diff_y = error_y - last_error_y;
    last_error_y = error_y;
    double v_y = P * error_y + D * diff_y;
    // Z****************************
    double error_z = std::max(sqrt(pow((p_squares[0].x - p_squares[1].x), 2) + pow((p_squares[0].y - p_squares[1].y), 2)),
                              sqrt(pow((p_squares[0].x - p_squares[3].x), 2) + pow((p_squares[0].y - p_squares[3].y), 2))) -
                     sqrt(pow((std_squares[0].x - std_squares[1].x), 2) + pow((std_squares[0].y - std_squares[1].y), 2));
    // static double integra_z += error_z;
    static double last_error_z;
    double diff_z = error_z - last_error_z;
    last_error_z = error_z;
    double v_z = P * error_z + D * diff_z;
    // wZ***************************
    static double last_error_wz;
    double diff_wz = angle - last_error_wz;
    last_error_wz = angle;
    double v_wz = P * angle * 10000 + D * diff_wz * 1000;

    // std::cout << v_x << " " << v_y << std::endl;
    std::cout << v_wz << std::endl;
    cmd_vel.data[0] = v_x;
    cmd_vel.data[1] = v_y;
    cmd_vel.data[2] = v_z;
    cmd_vel.data[5] = v_wz;
    if (p_mean.x < 1e-3 && p_mean.y < 1e-3)
    {
        cmd_vel.data[0] = 0;
        cmd_vel.data[1] = 0;
        cmd_vel.data[2] = 0;
        cmd_vel.data[5] = 0;
    }
    my_publisher.publish(cmd_vel);
    // std::vector<cv::DMatch> matches;
    // std::vector<std::vector<cv::DMatch>> knnMatches;
    // cv::BFMatcher bfMatcher(cv::NORM_HAMMING);
    // const float minRatio = 1.f / 1.5f;
    // if (squares.size() != 0)
    // {
    //     bfMatcher.match(p_squares, std_squares, matches);
    //     bfMatcher.knnMatch(p_squares, std_squares, knnMatches, 2);
    //     for (int i = 0; i < knnMatches.size(); i++)
    //     {
    //         const cv::DMatch &bestMatch = knnMatches[i][0];
    //         const cv::DMatch &betterMatch = knnMatches[i][1];
    //         float distanceRatio = bestMatch.distance / betterMatch.distance;
    //         if (distanceRatio < minRatio)
    //             matches.push_back(bestMatch);
    //     }
    // }
    // else
    // {
    // }
    // std::vector<cv::KeyPoint> keypoints_1;
    // cv::Mat descriptor_1;
    // cv::Mat sift_img;
    // cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10);
    // if (squares.size() != 0)
    // {
    //     sift->detectAndCompute(green_img, cv::noArray(), keypoints_1, descriptor_1);
    //     drawKeypoints(green_img, keypoints_1, sift_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    // }
    // else
    // {
    //     sift_img = green_img;
    // }
    // try
    // {
    // }
    // catch (...)
    // {
    //     std::cout << "555" << /*e.msg <<*/ std::endl;
    // }
    // for (int i = 0; i < msg_img.size(); i++)
    // {
    //     msg.data.push_back(msg_img[i].x);
    //     msg.data.push_back(msg_img[i].y);
    // }
    // my_publisher.publish(msg);

    cv::line(imgimg, p_mean, std_mean, cv::Scalar(255, 0, 255), 5, cv::LINE_AA);
    cv::imshow("imgimg", imgimg);
    // cv::imshow("mask_img", mask_img);
    cv::imshow("green_img", green_img);
    // cv::imshow("scharr_img", scharr_img);
    cv::imshow("jiao_img", jiao_img);
    // cv::imshow("jiao_img0", jiao_img0);
    cv::waitKey(4);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imgimgimage");
    ros::NodeHandle n;
    for (int i = 0; i < 6; i++)
    {
        cmd_vel.data.push_back(0.0);
    }
    ros::Subscriber my_subscriber = n.subscribe("/probot_anno/camera/image_raw", 1, myCallback);
    my_publisher = n.advertise<std_msgs::Float64MultiArray>("/cmd_vel", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    for (int i = 0; i < 6; i++)
    {
        cmd_vel.data[i] = 0;
    }
    return 0;
}