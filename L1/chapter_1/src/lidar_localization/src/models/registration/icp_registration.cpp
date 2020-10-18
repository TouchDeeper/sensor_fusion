/*
 * @Description: ICP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include <lidar_localization/tools/tic_toc.h>
#include "lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

ICPRegistration::ICPRegistration(const YAML::Node& node)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT, float>()) {
    
    float max_distance = node["max_distance"].as<float>();
    int num_ransac = node["num_ransac"].as<int>();
    float trans_eps = node["trans_eps"].as<float>();
    float eucli_eps = node["eucli_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_distance, num_ransac, trans_eps, eucli_eps, max_iter);
}

ICPRegistration::ICPRegistration(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT, float>()) {

    SetRegistrationParam(max_distance, num_ransac, trans_eps, eucli_eps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter) {
    icp_ptr_->setMaxCorrespondenceDistance(max_distance);
    icp_ptr_->setEuclideanFitnessEpsilon(eucli_eps);
//    icp_ptr_->setRANSACIterations(num_ransac);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setMaximumIterations(max_iter);
//    icp_ptr_->setRANSACOutlierRejectionThreshold(max_distance);

    LOG(INFO) << "ICP 的匹配参数为：" << std::endl
              << "max_distance: " << max_distance << ", "
              << "eucli_eps: " << eucli_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << ", "
              << "num_ransac" << num_ransac
              << std::endl << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);

    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    td::TicToc timer;
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = icp_ptr_->getFinalTransformation();
    align_time += timer.tos();
    align_count += 1;
    align_mean_time();
    return true;
}
}