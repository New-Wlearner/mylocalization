//
// Created by dongxiao on 2020/12/11.
//

#ifndef MYLOCALIZATION_REGISTRATION_INTERFACE_H
#define MYLOCALIZATION_REGISTRATION_INTERFACE_H
#include "mylocalization/sensor_data/cloud_data.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace mylocalization{
    class RegistrationInterface{
        // can use base to initial son, can delete son when delete base
        virtual ~RegistrationInterface() = default;
        virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target);
        virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                          const Eigen::Matrix4f& predict_pose,
                          const CloudData::CLOUD_PTR& target_cloud_ptr,
                          Eigen::Matrix4f& result_pose);
    };
}
#endif //MYLOCALIZATION_REGISTRATION_INTERFACE_H
