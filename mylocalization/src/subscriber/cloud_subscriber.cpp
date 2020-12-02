#include "mylocalization/subscriber/cloud_subscriber.h"

#include "glog/raw_logging.h"

namespace mylocalization
{
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string &topic_name, size_t buff_size) {
        subscriber_ = nh_.subscriber(topic_name,buff_size,&CloudSubscriber::msg_callback,this);
    }
    void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr )
    {
        CloudData cloud_data;
        cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr,*(cloud_data.cloud_ptr));
        new_cloud_data_.push_back(cloud_data);
    }
    void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff)
    {
        if(new_cloud_data_.size() > 0){
            cloud_data_buff.insert(cloud_data_buff.end(),new_cloud_data.begin(),new_cloud_data.end());
            new_cloud_data.clear();
        }


    }
}