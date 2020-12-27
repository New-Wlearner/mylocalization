#ifndef MYLOCALIZATION_SENSOR_DATA_GNSS_DATA_H_
#define MYLOCALIZATION_SENSOR_DATA_GNSS_DATA_H_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"

namespace mylocalization
{
    class GNSSData
    {
    private:
        // ?
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;
    public:
      double time = 0.0;
      double longtitude = 0.0;
      double latitude = 0.0;
      double altitude = 0.0;
      double local_E = 0.0;
      double local_N = 0.0;
      double local_U = 0.0;
      // GNSS的状态
      int status = 0;
      int service = 0;  

      void InitOriginPosition();
      void UpdateXYZ();
      static bool SyncData(std::deque<GNSSData>& UnsyncedData,std::deque<GNSSData>& SyncedData, double sync_time);
    };
    
}



#endif