/**
 * @file cloud_data.h
 * @author dongxiao
 * @brief  激光点云数据
 * @version 0.1
 * @date 2020-11-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef MYLOCALIZATION_SENSOR_DATA_CLOUD_DATA_H_
#define MYLOCALIZATION_SENSOR_DATA_CLOUD_DATA_H_

#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>

namespace mylocalization
{
class CloudData
{
private:
    /* data */
public:
    CloudData(/* args */):cloud_ptr(new CLOUD);
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

    double time = 0.0;
    CLOUD_PTR cloud_ptr;

};
    
}


#endif