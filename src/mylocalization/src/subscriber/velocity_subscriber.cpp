//
// Created by dongxiao on 2020/12/10.
//
#include "mylocalization/subscriber/velocity_subscriber.h"

namespace mylocalization{
    VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size):nh_(nh) {
        subscriber_ = nh.subscribe(topic_name,buff_size,&VelocitySubscriber::msg_callback, this);
    }
    void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr) {
        VelocityData velocity_data;
        velocity_data.time = twist_msg_ptr->header.stamp.toSec();
        velocity_data.angle_velocity.x = twist_msg_ptr->twist.angular.x;
        velocity_data.angle_velocity.y = twist_msg_ptr->twist.angular.y;
        velocity_data.angle_velocity.z = twist_msg_ptr->twist.angular.z;

        velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
        velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
        velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

        new_velocity_data_.push_back(velocity_data);
    }
    void VelocitySubscriber::ParseData(std::deque<VelocityData> &deque_velocity_data) {
        if(new_velocity_data_.size() > 0){
            deque_velocity_data.insert(deque_velocity_data.end(),new_velocity_data_.begin(),new_velocity_data_.end());
            new_velocity_data_.clear();
        }
    }
}
