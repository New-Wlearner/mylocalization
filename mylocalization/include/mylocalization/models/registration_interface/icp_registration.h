//
// Created by dongxiao on 2020/12/12.
//

#ifndef MYLOCALIZATION_ICP_REGISTRATION_H
#define MYLOCALIZATION_ICP_REGISTRATION_H
#include "mylocalization/models/registration_interface/registration_interface.h"
#include <pcl-1.8/pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>
#include "mylocalization/sensor_data/cloud_data.h"
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
namespace mylocalization{
    class ICPRegistration{
    public:
        ICPRegistration(YAML::Node& node);
        ICPRegistration(float eucli_eps, float max_dist, float trans_eps, int max_iter);
        bool SetInputTarget(CloudData::CLOUD_PTR& input_source);
        bool ScanMatch(CloudData::CLOUD_PTR& input_source,
                       Eigen::Matrix4f& predict_pose,
                       CloudData::CLOUD_PTR& result_cloud_ptr,
                       Eigen::Matrix4f& result_pose);
//        bool VisualScanMatch(CloudData::CLOUD_PTR& input_source,
//                             Eigen::Matrix4f& predict_pose,
//                             CloudData::CLOUD_PTR& result_cloud_ptr,
//                             Eigen::Matrix4f& result_pose);
    private:
        bool SetRegistrationParam(float eucli_eps, float max_dist, float trans_eps, int max_iter);
//        void KeyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing);
//        bool next_iter = false;
        pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> view{new pcl::visualization::PCLVisualizer("icp_visual")};



    };
}

#endif //MYLOCALIZATION_ICP_REGISTRATION_H
