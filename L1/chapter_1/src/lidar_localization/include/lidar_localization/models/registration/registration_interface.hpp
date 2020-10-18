/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <rosconsole/macros_generated.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual void align_mean_time(){
        float cur_mean_tiem = align_time / align_count;
        if(std::abs(mean_time - cur_mean_tiem)/cur_mean_tiem>0.2){
            mean_time = cur_mean_tiem;
            std::cout<<"align time : "<<align_time<<", "<<"align cout : "<<align_count<<std::endl;
            std::cout<<"mean align time : "<< mean_time<<std::endl;
        }

    }
    float align_time = 0;
    int align_count = 0;
    float mean_time = 0;
};
} 

#endif