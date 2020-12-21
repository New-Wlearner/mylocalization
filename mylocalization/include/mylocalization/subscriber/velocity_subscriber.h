//
// Created by dongxiao on 2020/12/10.
//

#ifndef MYLOCALIZATION_VELOCITY_SUBSCRIBER_H
#define MYLOCALIZATION_VELOCITY_SUBSCRIBER_H
#include "mylocalization/sensor_data/velocity_data.h"
#include <ros/ros.h>
#include <deque>
// 位姿相关的消息类型
#include "geometry_msgs/TwistStamped.h"
namespace mylocalization{
    class VelocitySubscriber{
    public:
        VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        VelocitySubscriber() = default;
        void ParseData(std::deque<VelocityData>& deque_velocity_data);

    private:
        void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<VelocityData> new_velocity_data_;

    };

}

#endif //MYLOCALIZATION_VELOCITY_SUBSCRIBER_H
