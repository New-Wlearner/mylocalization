//
// Created by dongxiao on 2020/12/11.
//

#ifndef MYLOCALIZATION_CLOUD_FILTER_INTERFACE_H
#define MYLOCALIZATION_CLOUD_FILTER_INTERFACE_H
#include <yaml-cpp/yaml.h>
#include "mylocalization/sensor_data/cloud_data.h"

namespace mylocalization{
    class CloudFilterInterface{
    public:
        // 因为会有类似voxel_filter等类似的很多fliter methods,所以这用虚函数
        virtual ~CloudFilterInterface() = default;
        virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,CloudData::CLOUD_PTR& filtered_cloud_ptr);
    };

}


#endif //MYLOCALIZATION_CLOUD_FILTER_INTERFACE_H
