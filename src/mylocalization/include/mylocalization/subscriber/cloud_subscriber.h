#ifndef MYLOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define MYLOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// 这个应该是把点云从ros中变到pcl中
#include <pcl_conversions/pcl_conversions.h>

#include "mylocalization/sensor_data/cloud_data.h"


namespace mylocalization
{
    class CloudSubscriber
    {
    private:
        ros::Subscriber subscriber_;
        ros::NodeHandle nh_;
        std::deque<CloudData> new_cloud_data_;
        void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
    public:
        // 声明带参数的构造函数后用default定义不带参数的构造函数
        CloudSubscriber() = default;
        CloudSubscriber(ros::NodeHandle& nh, std::string& topic_name, size_t buff_size);
        void ParseData(std::deque<CloudData>& deque_cloud_data);
        ~CloudSubscriber();
    };
    

}





#endif