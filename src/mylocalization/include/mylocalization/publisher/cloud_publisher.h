//
// Created by dongxiao on 2020/12/18.
//

#ifndef MYLOCALIZATION_CLOUD_PUBLISHER_H
#define MYLOCALIZATION_CLOUD_PUBLISHER_H
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "mylocalization/sensor_data/cloud_data.h"
namespace mylocalization{
    class CloudPublisher{
    public:
        CloudPublisher(ros::NodeHandle& nh,
                       std::string topic_name,
                       size_t buff_size,
                       std::string frame_id
                       );
        CloudPublisher() = default;
        void Publish(CloudData::CLOUD_PTR cloud_ptr_input);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;

    };
}

#endif //MYLOCALIZATION_CLOUD_PUBLISHER_H
