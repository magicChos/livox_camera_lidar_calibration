#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIRGB
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    PCL_ADD_RGB;
    union
    {
        struct
        {
            float intensity;
        };
        float data_c[4];
    };
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGB,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity))