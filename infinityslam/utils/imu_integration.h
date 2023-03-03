/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_UTILS_INTERNAL_3D_IMU_INTEGRATION_H_
#define INFINITYSLAM_UTILS_INTERNAL_3D_IMU_INTEGRATION_H_

#include <algorithm>
#include <deque>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/transform.h"
#include "infinityslam/transform/timed_pose.h"
#include "glog/logging.h"

namespace infinityslam {
namespace utils {

template <typename T>
struct IntegrateImuResult {
    Eigen::Matrix<T, 3, 1> delta_velocity;
    Eigen::Matrix<T, 3, 1> delta_translation;
    Eigen::Quaternion<T> delta_rotation;
};

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuResult<T> IntegrateImu(
    const RangeType& imu_data,
    const Eigen::Transform<T, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<T, 3, Eigen::Affine>& angular_velocity_calibration,
    const common::Time start_time, 
    const common::Time end_time,
    IteratorType* const it) 
{
    CHECK_LE(start_time, end_time);
    CHECK(*it != imu_data.end());
    CHECK_LE((*it)->time, start_time);
    if (std::next(*it) != imu_data.end()) {
        CHECK_GT(std::next(*it)->time, start_time);
    }

    common::Time current_time = start_time;

    IntegrateImuResult<T> result = {Eigen::Matrix<T, 3, 1>::Zero(),
                                    Eigen::Matrix<T, 3, 1>::Zero(),
                                    Eigen::Quaterniond::Identity().cast<T>()};
    while (current_time < end_time) {
        common::Time next_imu_data = common::Time::max();
        if (std::next(*it) != imu_data.end()) {
            next_imu_data = std::next(*it)->time;
        }
        common::Time next_time = std::min(next_imu_data, end_time);
        const T delta_t(common::ToSeconds(next_time - current_time));

        const Eigen::Matrix<T, 3, 1> delta_angle =
            (angular_velocity_calibration *
            (*it)->angular_velocity.template cast<T>()) *
            delta_t;
        result.delta_rotation *=
            transform::AngleAxisVectorToRotationQuaternion(delta_angle);
        result.delta_velocity += result.delta_rotation *
                                ((linear_acceleration_calibration *
                                (*it)->linear_acceleration.template cast<T>()) *
                                delta_t);
        result.delta_translation += delta_t * result.delta_velocity;
        current_time = next_time;
        if (current_time == next_imu_data) {
            ++(*it);
        }
    }
    return result;
}

// Returns velocity delta in map frame.
template <typename RangeType, typename IteratorType>
IntegrateImuResult<double> IntegrateImu(
    const RangeType& imu_data,
    const common::Time start_time,
    const common::Time end_time,
    IteratorType* const it) 
{
    return IntegrateImu<double, RangeType, IteratorType>(
        imu_data, Eigen::Affine3d::Identity(), 
        Eigen::Affine3d::Identity(),
        start_time, end_time, it);
}


// 专门用于计算旋转积分并支持对旋转积分按比例缩放的数据结构。
template<typename T>
struct RotationIncrement {
    Eigen::Matrix<T, 3, 1> delta_angle;
    Eigen::Quaternion<T> orientation;
};

// 执行按【比例】缩放旋转量
template <typename RangeType, typename IteratorType>
void ReScalingRotation(
    RangeType& rotation_data, 
    IteratorType* const it,
    const double& scale_factor) 
{
    //
}

// 执行按【误差均分补偿】调整旋转量
template <typename T, typename RangeType, typename IteratorType>
void CompensateRotationError(
    RangeType& rotation_data,
    IteratorType* const it,
    const Eigen::Quaternion<T>& rot_error) 
{
    //
}

/**
 * 在上述基础上，支持输出所有imu时刻的姿态。
 * 要求：
 * -- start_time 小于 end_time；
 * -- 当前迭代器指向数据时间戳小于 start_time；
 * -- 当前迭代器位置下一个数据的时间戳大于 start_time；
 * 
 * TODO: 引入模板类型T for float/double etc.
*/
template <typename T, typename RangeType, typename IteratorType>
bool IntegrateImuRotation(
    const RangeType& imu_data,
    const double& start_time,
    const double& end_time,
    IteratorType* const it,
    std::vector<RotationIncrement<T>>& rotations) 
{
    if (start_time >= end_time) {
        LOG(ERROR) << "start_time must be early than end_time!";
        return false;
    }
    if (*it == imu_data.end()) {
        LOG(ERROR) << "iterator must not be equal with `imu_data.end()`.";
        return false;
    }

    // if (std::next(*it) != imu_data.end()) {
    //     if (std::next(*it)->timestamp < start_time) {
    //         LOG(ERROR) << "next-iterator element must be later then start_time.";
    //         return false;
    //     }
    // }
    // if (imu_data.rbegin()->timestamp < end_time) {
    //     LOG(ERROR) << "tail element of imu_data must be later than end_time.";
    //     return false;
    // }
    if ((*it)->timestamp - start_time > 0.5) {
        LOG(WARNING) << "Beginning imu data is later than start_time for " 
            << (*it)->timestamp - start_time << " seconds (>" << 0.5 << ")";
    }

    rotations.clear();
    double current_time = start_time;
    Eigen::Quaternion<T> current_rotation = Eigen::Quaternion<T>::Identity();
    size_t counts = 0;
    while ((*it) != imu_data.end()) {
        const double next_time = (*it)->timestamp;
        Eigen::Quaternion<T> next_rotation = current_rotation;
        if (next_time > start_time) {
            const double delta_t = next_time - current_time;
            const Eigen::Matrix<T, 3, 1> delta_angle =
                (*it)->angular_velocity.template cast<T>() * delta_t;
            next_rotation = current_rotation *
                transform::AngleAxisVectorToRotationQuaternion(delta_angle);
            rotations.emplace_back(RotationIncrement<T>{delta_angle, next_rotation});
            ++counts;
            // LOG(INFO) << "integrate imu : one imu data used -- " << counts;
        }
        if (next_time > end_time) {
            if (!rotations.empty()) {
                rotations.pop_back();
                const double delta_t = end_time - current_time;
                const Eigen::Matrix<T, 3, 1> delta_angle =
                    (*it)->angular_velocity.template cast<T>() * delta_t;
                next_rotation = current_rotation *
                    transform::AngleAxisVectorToRotationQuaternion(delta_angle);
                rotations.emplace_back(RotationIncrement<T>{delta_angle, next_rotation});
            }
            break;
        }
        current_time = next_time;
        current_rotation = next_rotation;
        ++(*it);
    }

    if (rotations.size() == counts) return true;
    LOG(ERROR) << "unexpected error occured!";
    return false;
}

template <typename T>
struct ExtrapolatePoseResult {
    transform::Rigid3<T> pose;
    Eigen::Matrix<T, 3, 1> velocity;
};

// Returns pose and linear velocity at 'time' which is equal to
// 'prev_from_tracking' extrapolated using IMU data.
template <typename T, typename RangeType, typename IteratorType>
ExtrapolatePoseResult<T> ExtrapolatePoseWithImu(
    const transform::Rigid3<T>& prev_from_tracking,
    const Eigen::Matrix<T, 3, 1>& prev_velocity_in_tracking,
    const common::Time prev_time, const Eigen::Matrix<T, 3, 1>& gravity,
    const common::Time time, const RangeType& imu_data,
    IteratorType* const imu_it) 
{
    const IntegrateImuResult<T> result =
        IntegrateImu(imu_data, Eigen::Transform<T, 3, Eigen::Affine>::Identity(),
                    Eigen::Transform<T, 3, Eigen::Affine>::Identity(), prev_time,
                    time, imu_it);

    const T delta_t = static_cast<T>(common::ToSeconds(time - prev_time));
    const Eigen::Matrix<T, 3, 1> translation =
        prev_from_tracking.translation() +
        prev_from_tracking.rotation() *
            (delta_t * prev_velocity_in_tracking + result.delta_translation) -
        static_cast<T>(.5) * delta_t * delta_t * gravity;
    const Eigen::Quaternion<T> rotation =
        prev_from_tracking.rotation() * result.delta_rotation;

    const Eigen::Matrix<T, 3, 1> velocity =
        prev_from_tracking.rotation() *
            (prev_velocity_in_tracking + result.delta_velocity) -
        delta_t * gravity;
    return ExtrapolatePoseResult<T>{transform::Rigid3<T>(translation, rotation),
                                    velocity};
}

// Same as above but given the last two poses.
template <typename T, typename RangeType, typename IteratorType>
ExtrapolatePoseResult<T> ExtrapolatePoseWithImu(
    const transform::Rigid3<T>& prev_from_tracking,
    const common::Time prev_time,
    const transform::Rigid3<T>& prev_prev_from_tracking,
    const common::Time prev_prev_time, const Eigen::Matrix<T, 3, 1>& gravity,
    const common::Time time, const RangeType& imu_data,
    IteratorType* const imu_it) 
{
    // TODO(danielsievers): Really we should integrate velocities starting from
    // the midpoint in between two poses, since this is how we fit them to poses
    // in the optimization.
    const T prev_delta_t =
        static_cast<T>(common::ToSeconds(prev_time - prev_prev_time));
    const Eigen::Matrix<T, 3, 1> prev_velocity_in_tracking =
        prev_from_tracking.inverse().rotation() *
        (prev_from_tracking.translation() -
        prev_prev_from_tracking.translation()) /
        prev_delta_t;

    return ExtrapolatePoseWithImu(
        prev_from_tracking, 
        prev_velocity_in_tracking,
        prev_time, gravity, 
        time, imu_data, imu_it);
}

}  // namespace utils
}  // namespace infinityslam

#endif  // INFINITYSLAM_UTILS_INTERNAL_3D_IMU_INTEGRATION_H_
