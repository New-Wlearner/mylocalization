//
// Created by dongxiao on 2020/12/2.
//
#include "mylocalization/subscriber/gnss_subscriber.h"

namespace mylocalization{
    GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name,buff_size,&GNSSSubscriber::msg_callback,this);
    }
    void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr) {
        GNSSData gnss_data;
        gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
        gnss_data.altitude = nav_sat_fix_ptr->altitude;
        gnss_data.latitude = nav_sat_fix_ptr->latitude;
        gnss_data.longtitude = nav_sat_fix_ptr->longitude;
        // nav_sat_fix_ptr中没有local_neu;
        gnss_data.status = nav_sat_fix_ptr->status.status;
        gnss_data.service = nav_sat_fix_ptr->status.service;

        new_gnss_data_.push_back(gnss_data);
    }
    void GNSSSubscriber::ParseData(std::deque<GNSSData> &deque_gnn_data) {
        if(new_gnss_data_.size()>0){
            deque_gnn_data.insert(deque_gnn_data.end(),new_gnss_data_.begin(),new_gnss_data_.end());
            new_gnss_data_.clear();
        }

    }
}

