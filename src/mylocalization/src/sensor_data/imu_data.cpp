/**
 * @file imu_data.cpp
 * @author dongxiao(you@domain.com)
 * @brief  imu数据结构
 * @version 0.1
 * @date 2020-12-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "mylocalization/sensor_data/imu_data.hpp"
#include <cmath>
#include "glog/logging.h"

namespace mylocalization
{
    Eigen::Matrix3f IMUData::GetOrientationMatrix()
    {
        Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z );
        Eigen::Matrix3f matrix = q.matrix().cast<float>();
        return matrix;
    }
    bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time)
    {
        while(UnsyncedData.size() >= 2)
        {
            if(UnsyncedData.front().time > sync_time)
                return false;
            if(UnsyncedData.at(1).time < sync_time)
            {
                UnsyncedData.pop_front();
                continue;
            }
            if(sync_time - UnsyncedData.front().time > 0.2)
                return false;
            if(UnsyncedData.at(1).time - sync_time > 0.2)
                return false;
        }
        if(UnsyncedData.size() < 2)
            return false;
        IMUData front_data = UnsyncedData.front();
        IMUData back_data = UnsyncedData.at(1);
        IMUData sync_data;

        double k = (back_data.time - front_data.time)/(sync_time - front_data.time);
        sync_data.time = sync_time;
        sync_data.angular_velocity.x =  (1-k) * front_data.angular_velocity.x + k  * back_data.angular_velocity.x;
        sync_data.angular_velocity.y =  (1-k) * front_data.angular_velocity.y + k  * back_data.angular_velocity.y;
        sync_data.angular_velocity.z =  (1-k) * front_data.angular_velocity.z + k  * back_data.angular_velocity.z;
        sync_data.linear_acceleration.x = (1-k)*front_data.linear_acceleration.x + k * back_data.linear_acceleration.x;
        sync_data.linear_acceleration.y = (1-k)*front_data.linear_acceleration.y + k * back_data.linear_acceleration.y;
        sync_data.linear_acceleration.z = (1-k)*front_data.linear_acceleration.z + k * back_data.linear_acceleration.z;
        sync_data.orientation.w = (1-k) * front_data.orientation.w + k * back_data.orientation.w;
        sync_data.orientation.x = (1-k) * front_data.orientation.x + k * back_data.orientation.x;
        sync_data.orientation.y = (1-k) * front_data.orientation.y + k * back_data.orientation.y;
        sync_data.orientation.z = (1-k) * front_data.orientation.z + k * back_data.orientation.z;
        // 插值完记住归一化四元数
        sync_data.orientation.Normlize();
        SyncedData.push_back(sync_data);
        return true;



    }

}

