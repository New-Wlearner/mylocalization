#ifndef MYLOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define MYLOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#include <deque>
#include_next <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "mylocalization/sensor_data/gnss_data.h"

namespace mylocalization{
    class GNSSSubscriber{
    public:
        GNSSSubscriber() = default;
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        void ParseData(std::deque<GNSSData>& deque_gnn_data);

    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<GNSSData> new_gnss_data_;
    };
}
#endif