//
// Created by dongxiao on 2020/12/11.
//

#ifndef MYLOCALIZATION_NDT_REGISTRATION_H
#define MYLOCALIZATION_NDT_REGISTRATION_H
#include "mylocalization/models/registration_interface/registration_interface.h"
#include <pcl/registration/ndt.h>
namespace mylocalization{
class NDTRegistration{
public:
    NDTRegistration(YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);
    bool SetInputTarget(CloudData::CLOUD_PTR& input_source);
    bool ScanMatch(
            CloudData::CLOUD_PTR& input_source,
            Eigen::Matrix4f& predict_pose,
            CloudData::CLOUD_PTR& target_cloud_ptr,
            Eigen::Matrix4f& result_pose);

private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);
    pcl::NormalDistributionsTransform<CloudData::POINT,CloudData::POINT>::Ptr ndt_ptr_;
};


}
#endif //MYLOCALIZATION_NDT_REGISTRATION_H
