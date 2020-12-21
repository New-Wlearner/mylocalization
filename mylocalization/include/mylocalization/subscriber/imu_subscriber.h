//
// Created by dongxiao on 2020/12/10.
//

#ifndef MYLOCALIZATION_IMU_SUBSCRIBER_H
#define MYLOCALIZATION_IMU_SUBSCRIBER_H
#include "mylocalization/sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"
#include <deque>
#include <ros/ros.h>

namespace mylocalization{
    class IMUSubscriber{
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& deque_imu_data);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<IMUData> new_imu_data_;
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    };
}

#endif //MYLOCALIZATION_IMU_SUBSCRIBER_H
