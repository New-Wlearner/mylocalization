#include "mylocalization/sensor_data/gnss_data.h"
#include "glog/logging.h"

// ！静态变量必须在类外初始化
bool mylocalization::GNSSData::or
bool mylocalization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian mylocalization::GNSSData::geo_converter;

namespace mylocalization
{
    
}
