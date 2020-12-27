//
// Created by dongxiao on 2020/12/18.
//

#ifndef MYLOCALIZATION_ODOMETRY_PUBLISHER_H
#define MYLOCALIZATION_ODOMETRY_PUBLISHER_H
#include <string>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace mylocalization{
    class OdometryPublisher{
    public:
        OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id,int buff_size);
        OdometryPublisher() = default;
        int Publish(Eigen::Matrix4d& transform_matrix );
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
    };
}

#endif //MYLOCALIZATION_ODOMETRY_PUBLISHER_H
