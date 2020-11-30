#include "mylocalization/sensor_data/gnss_data.h"
#include "glog/logging.h"

// ！静态变量必须在类外初始化
bool mylocalization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian mylocalization::GNSSData::geo_converter;


namespace mylocalization
{
    // 设置初始的位置
    void GNSSData::InitOriginPosition()
    {
        geo_converter.Reset(latitude, longtitude, altitude);
        origin_position_inited = true;
    }

    void GNSSData::UpdateXYZ()
    {
        if(!origin_position_inited)
            LOG(WARNING) << "GNSS not initial!";
        // 把经纬度和海拔变成XYZ
        geo_converter.Forward(latitude,longtitude,altitude,local_E,local_N,local_U);
    }
    bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData,double sync_time)
    {
        while (UnsyncedData.size() >= 2 )
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

        GNSSData front_data = UnsyncedData.front();
        GNSSData back_time = UnsyncedData.at(1);
        GNSSData sync_data;
        


        
        

    }

    
}
