#include "synchronizer.h"
#include "common.h"

Synchronizer::Synchronizer(const std::string cloud_topic, const std::string image_topic, const std::string &intrisc_path, const std::string &extrisc_path, const ros::NodeHandle &nh)
    : m_cloud_topic_name(cloud_topic),
      m_image_topic_name(image_topic),
      m_intrinsic_path(intrisc_path),
      m_extrinsic_path(extrisc_path),
      nh_(nh),
      private_node_("~")
{
    pubPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/Syn/lidar", 1);
    pubImage = nh_.advertise<sensor_msgs::Image>("/Syn/image", 1);

    initParams(m_intrinsic_path, m_extrinsic_path);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, m_image_topic_name, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh_, m_cloud_topic_name, 1);
    ROS_INFO("----------------------------");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, image_sub);
    sync.registerCallback(boost::bind(&Synchronizer::callback, this, _1, _2));

    ros::spin();
}

void Synchronizer::initParams(const string &intrinsic_path, const string &extrinsic_path)
{
    std::vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);

    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);

    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}};
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};

    // transform into the opencv matrix

    matrix_in = cv::Mat(3, 3, CV_64F, matrix1);
    matrix_out = cv::Mat(3, 4, CV_64F, matrix2);

    camera_matrix.at<double>(0, 0) = intrinsic[0];
    camera_matrix.at<double>(0, 2) = intrinsic[2];
    camera_matrix.at<double>(1, 1) = intrinsic[4];
    camera_matrix.at<double>(1, 2) = intrinsic[5];

    distortion_coef.at<double>(0, 0) = distortion[0];
    distortion_coef.at<double>(1, 0) = distortion[1];
    distortion_coef.at<double>(2, 0) = distortion[2];
    distortion_coef.at<double>(3, 0) = distortion[3];
    distortion_coef.at<double>(4, 0) = distortion[4];

    cv::Mat view, rview;
    cv::Size imageSize(640, 480);
    cv::initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(), cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coef, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
}

void Synchronizer::callback(const sensor_msgs::PointCloud2::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_image)
{
    cout << "*******************" << endl;
    sensor_msgs::PointCloud2 syn_pointcloud = *ori_pointcloud;
    sensor_msgs::Image syn_image = *ori_image;

    ROS_INFO("pointcloud stamp value is: %f", syn_pointcloud.header.stamp.toSec());
    ROS_INFO("image stamp value is: %f", syn_image.header.stamp.toSec());

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ori_pointcloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    cv::Mat src_img = cv_bridge::toCvShare(ori_image, "bgr8")->image;
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR); // correct the distortion

    int row = src_img.rows;
    int col = src_img.cols;

    vector<vector<int>> color_vector;
    color_vector.resize(row * col);
    for (unsigned int i = 0; i < color_vector.size(); ++i)
    {
        color_vector[i].resize(3);
    }

    // read photo and get all RGB information into color_vector
    ROS_INFO("Start to read the photo ");
    for (int v = 0; v < row; ++v)
    {
        for (int u = 0; u < col; ++u)
        {
            // for .bmp photo, the 3 channels are BGR
            color_vector[v * col + u][0] = src_img.at<cv::Vec3b>(v, u)[2];
            color_vector[v * col + u][1] = src_img.at<cv::Vec3b>(v, u)[1];
            color_vector[v * col + u][2] = src_img.at<cv::Vec3b>(v, u)[0];
        }
    }
    ROS_INFO("Finish saving the data ");

    // to do
    // 将RGB赋值给点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = syn_pointcloud.data.size();
    cloud->points.resize(cloud->width);
    for (uint64_t i = 0; i < cloud->points.size() && nh_.ok(); ++i)
    {
        float x = temp_cloud->points[i].x;
        float y = temp_cloud->points[i].y;
        float z = temp_cloud->points[i].z;

        // ignore the invalid point
        if (x == 0 && y == 0 && z == 0)
        {
            continue;
        }

        // set coordinate for the cloud point
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;

        // set the RGB for the cloud point
        int RGB[3] = {0, 0, 0};
        getColor(matrix_in, matrix_out, x, y, z, row, col, color_vector, RGB);
        // ignore the unexisting point
        if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0)
        {
            continue;
        }

        cloud->points[i].r = RGB[0];
        cloud->points[i].g = RGB[1];
        cloud->points[i].b = RGB[2];
    }

    sensor_msgs::PointCloud2 syn_pointcloud_output;
    pcl::toROSMsg(*cloud, syn_pointcloud_output);
    syn_pointcloud_output.header.frame_id = "livox_frame";
    syn_pointcloud_output.header.stamp = syn_pointcloud_output.header.stamp;

    // 发布彩色点云
    pubPointCloud.publish(syn_pointcloud_output);
    // pubImage.publish(syn_image);
}

// use extrinsic and intrinsic to get the corresponding U and V
void Synchronizer::getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float *UV)
{
    double matrix3[4][1] = {x, y, z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);

    // calculate the result of u and v
    cv::Mat result = matrix_in * matrix_out * coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    UV[0] = u / depth;
    UV[1] = v / depth;
}

void Synchronizer::getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int *RGB)
{
    float UV[2] = {0, 0};
    getUV(matrix_in, matrix_out, x, y, z, UV); // get U and V from the x,y,z

    int u = int(UV[0]);
    int v = int(UV[1]);

    int32_t index = v * col + u;
    if (index < row * col && index >= 0)
    {
        RGB[0] = color_vector[index][0];
        RGB[1] = color_vector[index][1];
        RGB[2] = color_vector[index][2];
    }
}