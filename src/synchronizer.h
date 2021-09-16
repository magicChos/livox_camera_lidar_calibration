/************************************************************************************************
@filename    :time_nchronizer.cpp
@brief       :image和pointcloud同步
@time        :2021/08/02 01:16:36
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/
#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "CustomPointCloud.h"

using namespace std;

class Synchronizer
{
public:
    Synchronizer(const std::string cloud_topic, const std::string image_topic, const std::string &intrisc_path , const std::string &extrisc_path , const ros::NodeHandle &nh);
    ~Synchronizer(){};
    void callback(const sensor_msgs::PointCloud2::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_image);

private:
    void initParams(const string &intrinsic_path, const string &extrinsic_path);
    void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float *UV);
    void getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int *RGB);

private:
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
    static cv::Mat matrix_in, matrix_out;
    std::string m_cloud_topic_name;
    std::string m_image_topic_name;
    std::string m_intrinsic_path;
    std::string m_extrinsic_path;
    ros::Publisher pubPointCloud;
    ros::Publisher pubImage;
    ros::NodeHandle nh_;
    ros::NodeHandle private_node_;
    cv::Mat map1, map2;
};

