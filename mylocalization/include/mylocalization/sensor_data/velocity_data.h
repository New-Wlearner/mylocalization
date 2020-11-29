#ifndef MYLOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H_
#define MYLOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H_

#include <deque>
#include <Eigen/Dense>

namespace mylocalization
{
    class VelocityData
    {
    private:
        /* data */
    public:
        struct LinearVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };
        struct AngleVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;

        };
        double time = 0.0;
        LinearVelocity linear_velocity;
        AngleVelocity angle_velocity;

        static bool SyncData(std::deque<VelocityData>& UnsyncData, std::deque<VelocityData>& SyncedData, double sync_time);
        
    };

    
}




#endif