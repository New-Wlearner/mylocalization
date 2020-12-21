//
// Created by dongxiao on 2020/12/12.
//
#include "mylocalization/models/registration_interface/registration_interface.h"
#include "mylocalization/models/registration_interface/ndt_registration.h"

namespace mylocalization{
    NDTRegistration::NDTRegistration(YAML::Node &node) {
        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<float>();
        SetRegistrationParam(res,step_size,trans_eps,max_iter);
    }
    NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter) {
        SetRegistrationParam(res,step_size,trans_eps,max_iter);
    }
    bool NDTRegistration::SetInputTarget(CloudData::CLOUD_PTR &input_source) {
        ndt_ptr_->setInputTarget(input_source);
        return true;
    }
    bool NDTRegistration::ScanMatch(CloudData::CLOUD_PTR &input_source, Eigen::Matrix4f &predict_pose,
                                    CloudData::CLOUD_PTR &target_cloud_ptr, Eigen::Matrix4f &result_pose) {
        ndt_ptr_->setInputSource(input_source);
        ndt_ptr_->align(*target_cloud_ptr,predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();

        return true;

    }
    bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);
        return true;
    }
}
