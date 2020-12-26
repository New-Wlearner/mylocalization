//
// Created by dongxiao on 2020/12/21.
//

#ifndef MYLOCALIZATION_FRONT_END_H
#define MYLOCALIZATION_FRONT_END_H
#include "mylocalization/sensor_data/cloud_data.h"
#include "mylocalization/models/registration_interface/ndt_registration.h"
#include "mylocalization/models/cloud_filter/voxel_filter.h"

namespace mylocalization{
    class FrontEnd{
    public:
        struct Frame{
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };
        FrontEnd();
        // initialization
        bool InitWithConfig();
        bool Update(const CloudData& cloud_data, Eigen::Matrix4f& pose);
        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool SaveMap();
        bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
        bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

    private:
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(YAML::Node& config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, YAML::Node& config_node);
        bool UpdateWithNewFrame(const Frame& new_key_frame);

        std::string data_path_ = "";
        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Frame current_frame_;
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_map_ptr_;

        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;

        std::deque<Frame> local_map_frames_;
        std::deque<Frame> global_map_frames_;

        float key_frame_distance_ = 2.0;
        int local_frame_nums = 20;

    };
}
#endif //MYLOCALIZATION_FRONT_END_H
