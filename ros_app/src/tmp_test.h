/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
*/

#ifndef ROSAPP_TMP_TEST_H_
#define ROSAPP_TMP_TEST_H_

#include <cmath>
#include <ctime>
#include <string>
#include <deque>
#include <vector>
#include <iomanip>
#include <thread>
#include <chrono>
#include <sstream>

#include <boost/format.hpp>
#include <boost/make_unique.hpp>
#include <Eigen/Core>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>


namespace ros_app {


void ConvertSO3ToRPY() {
    // data: [ 2.67949e-08, -1,  0, 0,
    //         1,  2.67949e-08,  0, 0,
    //         0,  0,  1, -0.28, 
    //         0., 0., 0., 1 ] 

    Eigen::Matrix3d so3;
    so3(0,0) = 2.67949e-08; so3(0,1) = -1;          so3(0,2) = 0;
    so3(1,0) = 1;           so3(1,1) = 2.67949e-08; so3(1,2) = 0;
    so3(2,0) = 0;           so3(2,1) = 0;           so3(2,2) = 1;
    LOG(INFO) << "Calculating transform ... \n" 
        << "From so3 to roll/pitch/yaw: \n" 
        << so3.eulerAngles(0,1,2) << " \n"
        << "From so3.inv to roll/pitch/yaw: \n" 
        << so3.inverse().eulerAngles(0,1,2)
        << "From so3.inv to roll/pitch/yaw (transpose): " 
        << so3.inverse().eulerAngles(0,1,2).transpose();

    Eigen::Vector3i vec3i {3,4,5};
    LOG(INFO) << "vector3i test: \n" 
        // << std::fixed << std::setprecision(2) 
        << vec3i << "\n" << vec3i + Eigen::Vector3i::Ones(); 
    LOG(INFO) << "vector3i test for vector3f: \n" 
        // << std::fixed << std::setprecision(2) 
        << vec3i.cast<float>() << "\n" << vec3i.cast<float>() + 0.7958154f * Eigen::Vector3f::Ones(); 
    LOG(INFO) << "vector3i test2 for vector3f: \n" 
        << (vec3i + Eigen::Vector3i::Ones()) * 3 << "\n" << (vec3i + Eigen::Vector3i::Ones()).cast<float>() * 1.111; 
}









} // namespace ros_app

#endif // ROSAPP_TMP_TEST_H_