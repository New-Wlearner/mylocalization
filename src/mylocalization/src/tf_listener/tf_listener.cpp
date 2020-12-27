//
// Created by dongxiao on 2020/12/10.
//
#include "mylocalization/tf_listener/tf_listener.h"

namespace mylocalization{
    TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id)
        :nh_(nh),base_frame_id_(base_frame_id),child_frame_id_(child_frame_id) {
    }
    bool TFListener::LookupData(Eigen::Matrix4f &transform_matrix) {
        try{
            tf::StampedTransform transform;
            listener_.lookupTransform(base_frame_id_,child_frame_id_,ros::Time(0),transform);
            TransformToMatrix(transform,transform_matrix);
            return true;
        }
        catch (tf::TransformException& e) {
            return false;

        }
    }
    bool TFListener::TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4f &transform_matrix) {
        // getOrigin can get translate
        Eigen::Translation3f tl_btol(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
        double roll,pitch,yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw,pitch,roll);
        // rotation_vector
        // angleaxisd * -> quaterniond
        // translation3f * quaterniond  -> ?
        Eigen::AngleAxisf rot_x_btol(roll,Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y_btol(pitch,Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
        transform_matrix = (tl_btol * rot_x_btol * rot_y_btol * rot_z_btol).matrix();
        return true;

    }
}