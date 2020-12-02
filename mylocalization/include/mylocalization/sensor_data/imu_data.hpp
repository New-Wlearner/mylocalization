#ifndef MYLOCALIZATION_SENSOR_DATA_IMU_DATA_H_
#define MYLOCALIZATION_SENSOR_DATA_IMU_DATA_H_

#include <deque>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace
{
    class IMUData
    {
    public:
        struct LinearAcceleration
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };
        struct AngularVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };
        class Orientation
        {
        public:
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
            
            void Normlize()
            {
                double norm = sqrt(pow(x,2.0)+pow(y,2.0)+pow(z,2.0)+pow(w,2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
            }
        };

        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;
        
        

        // 把四元数转换为旋转矩阵
        Eigen::Matrix3f GetOrientationMatrix();
        // ? sync_time
        static bool SyncData(std::deque<IMUData>& UnsyncData, std::deque<IMUData>& SyncedData, double sync_time);
    };
    
}




#endif