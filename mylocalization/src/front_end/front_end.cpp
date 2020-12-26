//
// Created by dongxiao on 2020/12/24.
//
#include "mylocalization/front_end/front_end.h"
#include "mylocalization/global_defination/global_defination.h"

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include "glog/logging.h"

namespace mylocalization{
    FrontEnd::FrontEnd() :
    local_map_ptr_(new CloudData::CLOUD_PTR()),
    global_map_ptr_(new CloudData::CLOUD_PTR()),
    result_map_ptr_(new CloudData::CLOUD_PTR()){
        InitWithConfig();
    }
    bool FrontEnd::InitWithConfig() {
        std::string config_data_path = WORK_SPACE_PATH + "config/front_end/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_data_path);

        InitDataPath(config_node);
        InitRegistration(registration_ptr_,config_node);
        InitFilter("local_map",local_filter_ptr_,config_node);
        InitFilter("frame",frame_filter_ptr_,config_node);
        InitFilter("display_map",display_filter_ptr_,config_node);

        return true;
    }
    bool FrontEnd::InitDataPath(YAML::Node &config_node) {
        data_path_ = config_node["data_path"].as<std::string>();

        if(data_path_ = "./")
            data_path_ = WORK_SPACE_PATH;
        data_path_ += "/slam_data";
        if(boost::filesystem::is_directory(data_path_))
            boost::filesystem::remove(data_path_);
        boost::filesystem::create_directory(data_path_);

        if(boost::filesystem::is_directory(data_path_))
            LOG(INFO) << "data path: " << data_path_;
        else{
            LOG(WARNING) << "data_path creat failed!";
            return false;
        }

        std::string data_path_keyframe = data_path_ + "/KeyFrame";
        if(boost::filesystem::is_directory(data_path_keyframe))
            boost::filesystem::remove(data_path_keyframe);
        boost::filesystem::create_directory(data_path_keyframe);

        if(boost::filesystem::is_directory(data_path_keyframe))
            LOG(INFO) << "data_path_keyframe: " << data_path_keyframe;
        else{
            LOG(WARNING) << "data_path_keyframe creat failed!";
            return false;
        }
        return true;

    }
    bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr,
                                    const YAML::Node &config_node){
        std::string registration_method = config_node["registration_method"].as<std::string>();
        if(registration_method == "NDT")
            registration_ptr = std::shared_ptr<NDTRegistration>(config_node[registration_method]);
        else{
            LOG(ERROR) << "can't find " << registration_method;
            return false;
        }
        return true;
    }

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr,
                          YAML::Node &config_node) {
        std::string filter_name = config_node[filter_user + "_filter"].as<std::string>();
        LOF(INFO) << "Filter method: " << filter_name;
        if(filter_name == "voxel_filter"){
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_name][filter_user]);
        }
        else{
            LOG(ERROR) << "Can't find " << filter_name << "method！"
            return false;
        }
        return true;
    }

    bool FrontEnd::UpdateWithNewFrame(const Frame &new_key_frame) {
        std::string file_path = data_path_ + "/KeyFrame/key_frame_" + std::to_string(global_map_frames_.size())+".pcd";
        pcl::io::savePCDFileBinary(file_path,*new_key_frame.cloud_data.cloud_ptr);

        Frame key_frame = new_key_frame;
        key_frame.cloud_data.cloud_ptr.reset(*new_key_frame.cloud_data.cloud_ptr);
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
        local_map_frames_.push_back(key_frame)
        while(local_map_frames_.size() > local_frame_nums){
            local_map_frames_.pop_front();
        }
        for(int i = 0; i < local_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                     *transformed_cloud_ptr,
                                     local_map_frames_.at(i).pose);
            *local_map_ptr_ += transformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        if(local_map_frames_.size() < 10){
            registration_ptr_->SetInputTarget(local_map_ptr_);
        }
        else{
            CloudData::CLOUD_PTR filtered_local_map_ptr
            local_filter_ptr_->Filter(local_map_ptr_,filtered_local_map_ptr);
            registration_ptr_->SetInputTarget(filtered_local_map_ptr);
        }
        key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
        global_map_frames_.push_back(key_frame);

        return true;

    }
    bool FrontEnd::Update(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose) {
        current_frame_.cloud_data.time = cloud_data.time;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data,*current_frame_.cloud_data,indices);

        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr,filtered_cloud_ptr);

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        if(local_map_frames_.size() == 0){
            current_frame_.pose = init_pose_;
            UpdateWithNewFrame(current_frame_);
            cloud_pose = current_frame_.pose;
            return true;
        }

        registration_ptr_->ScanMatch(filtered_cloud_ptr,predict_pose,result_map_ptr_,current_frame_.pose);
        cloud_pose = current_frame_.pose;

        step_pose = last_key_frame_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;

        if(fabs(last_key_frame_pose(0,3)-current_frame_.pose(0,3))+
        fabs(last_key_frame_pose(1,3)-current_frame_.pose(1,3))+
        fabs(last_key_frame_pose(2,3)-current_frame_.pose(2,3)) > key_frame_distance_){
            UpdateWithNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }
        return true;
    }

    bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose) {
        init_pose_ = init_pose;
        return true;
    }
}
