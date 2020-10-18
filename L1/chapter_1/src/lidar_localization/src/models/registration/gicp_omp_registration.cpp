/*
 * @Description: GICPOMP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/gicp_omp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

GICPOMPRegistration::GICPOMPRegistration(const YAML::Node& node)
    :gicp_omp_ptr_(new pclomp::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    float max_distance = node["max_distance"].as<float>();
    int num_ransac = node["num_ransac"].as<int>();
    float trans_eps = node["trans_eps"].as<float>();
    float eucli_eps = node["eucli_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();
    int opt_iter = node["opt_iter"].as<int>();
    SetRegistrationParam(max_distance, num_ransac, trans_eps, eucli_eps, max_iter, opt_iter);
}

GICPOMPRegistration::GICPOMPRegistration(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter, int opt_iter)
    :gicp_omp_ptr_(new pclomp::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(max_distance, num_ransac, trans_eps, eucli_eps, max_iter, opt_iter);
}

bool GICPOMPRegistration::SetRegistrationParam(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter, int opt_iter) {
    gicp_omp_ptr_->setMaxCorrespondenceDistance(max_distance);
    gicp_omp_ptr_->setEuclideanFitnessEpsilon(eucli_eps);
    gicp_omp_ptr_->setMaximumOptimizerIterations(opt_iter);
//    gicp_omp_ptr_->setRANSACIterations(num_ransac);
    gicp_omp_ptr_->setTransformationEpsilon(trans_eps);
    gicp_omp_ptr_->setMaximumIterations(max_iter);
//    gicp_omp_ptr_->setRANSACOutlierRejectionThreshold(max_distance);

    LOG(INFO) << "GICPOMP 的匹配参数为：" << std::endl
              << "max_distance: " << max_distance << ", "
              << "eucli_eps: " << eucli_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << ", "
              << "opt_iter: " << opt_iter
              << std::endl << std::endl;

    return true;
}

bool GICPOMPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    gicp_omp_ptr_->setInputTarget(input_target);

    return true;
}

bool GICPOMPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    gicp_omp_ptr_->setInputSource(input_source);
    gicp_omp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = gicp_omp_ptr_->getFinalTransformation();

    return true;
}
}