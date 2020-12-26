//
// Created by dongxiao on 2020/12/12.
//
#include "mylocalization/models/registration_interface/registration_interface.h"
#include "mylocalization/models/registration_interface/icp_registration.h"
namespace mylocalization{
    ICPRegistration::ICPRegistration(YAML::Node &node) {
        // sum of average  junfang error
        float eucli_eps = node["eucli_eps"].as<float>();
        // 对应点对之间的最大差值
        float max_dist = node["max_dist"].as<float>();
        // 两次变换矩阵的差值
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<float>();
        SetRegistrationParam(eucli_eps,max_dist,trans_eps,max_iter);
    }
    ICPRegistration::ICPRegistration(float eucli_eps, float max_dist, float trans_eps, int max_iter) {
        SetRegistrationParam(eucli_eps,max_dist,trans_eps,max_iter);
    }
    bool ICPRegistration::SetRegistrationParam(float eucli_eps, float max_dist, float trans_eps, int max_iter) {
        icp_ptr_->setEuclideanFitnessEpsilon(eucli_eps);
        icp_ptr_->setMaxCorrespondenceDistance(max_dist);
        icp_ptr_->setTransformationEpsilon(trans_eps);
        icp_ptr_->setMaximumIterations(max_iter);
        return true;
    }
    bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source, const Eigen::Matrix4f &predict_pose,
                                    const CloudData::CLOUD_PTR &result_cloud_ptr, Eigen::Matrix4f &result_pose) {
        icp_ptr_->setInputSource(input_source);
        icp_ptr_->align(*result_cloud_ptr,predict_pose);
        result_pose = icp_ptr_->getFinalTransformation();
        return true;
    }
    bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_source) {
        icp_ptr_->setInputTarget(input_source);
        return true;
    }
    // void ICPRegistration::KeyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing) {
    //     if(event.getKeySym() == "space" && event.keyDown())
    //         next_iter = true;
    // }
    // https://segmentfault.com/a/1190000005930422
//    bool ICPRegistration::VisualScanMatch(CloudData::CLOUD_PTR &input_source, Eigen::Matrix4f &predict_pose,
//                                          CloudData::CLOUD_PTR &result_cloud_ptr, Eigen::Matrix4f &result_pose) {
//        int v1;
//        int v2;
//        CloudData::CLOUD_PTR Final;
//        // range
//        view->createViewPort(0.0,0.0,30,50,v1);
//        view->createViewPort(0.0,0.0,30,50,v2);
//        // input_cloud red
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color(input_source,255,0,0);
//        view->addPointCloud(input_source,source_cloud_color,"input_cloud",v1);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(result_cloud_ptr,0,255,0);
//        view->addPointCloud(result_cloud_ptr,target_cloud_color,"target_cloud",v1);
//        // background color
//        view->setBackgroundColor(0.0,0.05,0.05,v1);
//        view->setBackgroundColor(0.0,0.05,0.05,v2);
//        // point size
//        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"input_cloud");
//        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud");
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> align_cloud_color(Final,255,255,255);
//        view->addPointCloud(Final,align_cloud_color,"align_cloud",v2);
//        view->addPointCloud(result_cloud_ptr,target_cloud_color,"target_cloud_v2",v2);
//
//        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"align_cloud");
//        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");
//
//        view->registerKeyboardCallback(&ICPRegistration::KeyboardEvent, (void*)NULL);
//
//        int iter = 0;
//        while(!view->wasStopped())
//        {
//            view->spinOnce();
//            if(next_iter){
//                icp_ptr_->align(*Final);
//                // hasConverged return 1 means registration success
//                cout<<"has conveged: "<<icp_ptr_->hasConverged()<<endl<<
//                    "score: "<<icp_ptr_->getFitnessScore()<<endl;
//                cout<<"Transform Matrix: "<<icp_ptr_->getFinalTransformation()<<endl;
//                cout<<"iterations: "<<iter++<<endl;
//                if(iter == 100)
//                    return 0;
//                view->updatePointCloud(Final,align_cloud_color,"align_cloud");
//
//            }
//        }



//    }
}
