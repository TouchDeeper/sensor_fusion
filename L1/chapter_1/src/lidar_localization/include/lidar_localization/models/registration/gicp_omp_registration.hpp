/*
 * @Description: GICPOMP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_GICPOMP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_GICPOMP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/pclomp/gicp_omp.h"
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class GICPOMPRegistration: public RegistrationInterface {
  public:
    GICPOMPRegistration(const YAML::Node& node);
    GICPOMPRegistration(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter, int opt_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float max_distance, int num_ransac, float trans_eps, float eucli_eps, int max_iter, int opt_iter);

  private:
    pclomp::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr gicp_omp_ptr_;
};
}

#endif