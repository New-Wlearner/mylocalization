//
// Created by dongxiao on 2020/12/11.
//

#ifndef MYLOCALIZATION_VOXEL_FILTER_H
#define MYLOCALIZATION_VOXEL_FILTER_H
#include "cloud_filter_interface.h"
#include <pcl/filters/voxel_grid.h>

namespace mylocalization{
    class VoxelFilter{
    public:
        VoxelFilter(const YAML::Node& node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr);

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };
}

#endif //MYLOCALIZATION_VOXEL_FILTER_H
