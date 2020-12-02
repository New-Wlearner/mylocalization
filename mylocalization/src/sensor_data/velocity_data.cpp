#include "mylocalization/sensor_data/velocity_data.h"
#include "glog/logging.h"

namespace mylocalization
{
    bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncdData, std::deque<VelocityData>& SyncedData, double sync_time)
    {
        while (UnsyncdData.size() >= 2 )
        {
            if(UnsyncdData.front().time > sync_time)
                return false;
            if(UnsyncdData.at(1).time < sync_time)
            {
                UnsyncdData.pop_front();
                continue;
            }
            if(sync_time - UnsyncdData.front().time > 0.2)
                return false;
            if(UnsyncdData.at(1) - sync_time > 0.2)
                return false;
        }
        if(UnsyncdData.size() < 2)
            return false;
        VelocityData front_data = UnsyncdData.front();
        VelocityData back_data = UnsyncdData.at(1);
        VelocityData sync_data;
        double k = (back_data.time - front_data.time)/(sync_time - front_data.time);
        sync_data.time = sync_time;
        sync_data.angle_velocity.x = (1-k)*front_data.angle_velocity.x+k*back_data.angle_velocity.x;
        sync_data.angle_velocity.y = (1-k)*front_data.angle_velocity.y+k*back_data.angle_velocity.y;
        sync_data.angle_velocity.z = (1-k)*front_data.angle_velocity.z+k*back_data.angle_velocity.z;
        sync_data.linear_velocity.x = (1-k)*front_data.linear_velocity.x + k*back_data.linear_velocity.x;
        sync_data.linear_velocity.y = (1-k)*front_data.linear_velocity.y + k*back_data.linear_velocity.y;
        sync_data.linear_velocity.z = (1-k)*front_data.linear_velocity.z + k*back_data.linear_velocity.z;

        SyncedData.push_back(sync_data);
        return true;
        
    }
    
} // namespace mylocalization
