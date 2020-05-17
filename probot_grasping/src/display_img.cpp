#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include "std_msgs/Float64MultiArray.h"

ros::Publisher my_publisher;
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
    std::vector<std::vector<cv::Point> > squares;
    std::vector<cv::Point> p_squares;
    std::vector<std::vector<cv::Point> > contours;
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
            if (maxCosine < 0.3) {squares.push_back(approx); }                
        }
    }
    if (squares.size() != 0) 
    {
        std::cout << squares[0].size() << "个角点" << std::endl;
        for (int i = 0; i < squares[0].size(); i++)
        {
            p_squares.push_back(squares[0][i]);
        }
    }
    else
    {
        std::cout << 0  << "个角点" << std::endl; 
        for (int i = 0; i < 4; i++)
        {
            p_squares.push_back(cv::Point(0, 0));
        }
    }

    const cv::Point *p = &p_squares[0];
    int n = (int)p_squares.size();
    if (p->x > 3 && p->y > 3)
    {
        // std::cout << p->x << std::endl; 
        cv::polylines(jiao_img, &p, &n, 1, true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::polylines(imgimg, &p, &n, 1, true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

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

    cv::imshow("imgimg", imgimg);
    cv::imshow("mask_img", mask_img);
    cv::imshow("green_img", green_img);
    cv::imshow("scharr_img", scharr_img);
    cv::imshow("jiao_img", jiao_img);
    cv::imshow("jiao_img0", jiao_img0);
    cv::waitKey(100);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imgimgimage");
    ros::NodeHandle n;
    ros::Subscriber my_subscriber = n.subscribe("/probot_anno/camera/image_raw", 10, myCallback);
    my_publisher = n.advertise<std_msgs::Float64MultiArray>("/display_img", 10);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}