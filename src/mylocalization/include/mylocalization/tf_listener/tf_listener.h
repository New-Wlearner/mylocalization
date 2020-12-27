//
// Created by dongxiao on 2020/12/10.
//

#ifndef MYLOCALIZATION_TF_LISTENER_H
#define MYLOCALIZATION_TF_LISTENER_H
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace mylocalization{
    class TFListener{
    public:
        TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;
        bool LookupData(Eigen::Matrix4f& transform_matrix);

    private:
        bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;

    };
}


#endif //MYLOCALIZATION_TF_LISTENER_H
