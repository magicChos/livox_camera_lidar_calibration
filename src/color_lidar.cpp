

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <cmath>
#include <hash_map>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include "synchronizer.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;
int threshold_lidar;
string input_photo_path, input_bag_path, intrinsic_path, extrinsic_path;

void getParameters()
{
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("intrinsic_path", intrinsic_path))
    {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path))
    {
        cout << "Can not get the value o以下程序节点中如果想修改launch文件，需要到src/calibration/launch文件夹中找对应的launch文件。f extrinsic_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_lidar");
    ros::NodeHandle n;
    getParameters();

    ROS_INFO("\033[1;32m---->\033[0m Sync msgs node Started.");
    std::string cloud_topic = "/livox/lidar";
    std::string image_topic = "/camera/image";

    Synchronizer wode(cloud_topic, image_topic, intrinsic_path, extrinsic_path, n);
    return 0;
}
