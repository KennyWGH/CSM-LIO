/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#include "infinityslam/utils/imu_aided_pose_interpolator.h"
#include "glog/logging.h"

#include <cmath>
#include <iterator>
#include <iomanip>

namespace infinityslam {
namespace utils {

namespace {
} // namespace


ImuAidedPoseInterpolator::ImuAidedPoseInterpolator(double time_range_limit)
    :kTimeRangeLimit(time_range_limit) {}

void ImuAidedPoseInterpolator::AddPose(const transform::TimedPose& pose) {
    UniqLock lock(all_data_mutex_);
    if (!source_pose_queue_.empty() 
        && pose.timestamp < source_pose_queue_.back().timestamp) {
        return;
    }
    source_pose_queue_.push_back(pose);
    UniqLock lock2(interp_data_mutex_);
    AdvanceInterpolation();
    TrimSourcePoseQueueIfNeeded();
}

void ImuAidedPoseInterpolator::AddPose(
    const common::Time& time, 
    const transform::Rigid3d& local_pose) {
    UniqLock lock(all_data_mutex_);
    double timestamp = common::ToSeconds(time);
    if (!source_pose_queue_.empty() 
        && timestamp < source_pose_queue_.back().timestamp) {
        return;
    }
    source_pose_queue_.push_back(transform::TimedPose{time, timestamp, local_pose});
    UniqLock lock2(interp_data_mutex_);
    AdvanceInterpolation();
    TrimSourcePoseQueueIfNeeded();
}

void ImuAidedPoseInterpolator::AddImu(const sensor::ImuData& imu_data) {
    UniqLock lock(all_data_mutex_);
    if (!source_imu_queue_.empty() 
        && imu_data.timestamp < source_imu_queue_.back().timestamp) {
        return;
    }
    source_imu_queue_.push_back(imu_data);
    while (source_imu_queue_.size() > 200 && 
        (source_imu_queue_.back().timestamp 
            - source_imu_queue_.front().timestamp) > kTimeRangeLimit) 
    {
        source_imu_queue_.pop_front();
    }
}

// ***************************************************************************

// 支持单时刻内插查询
bool ImuAidedPoseInterpolator::LookUp(
    const double& timepoint, transform::TimedPose& pose) {
    UniqLock lock(interp_data_mutex_);
    // TODO
    return false;
}

// 支持时间段内按时间步长内插查询
bool ImuAidedPoseInterpolator::LookUp(
    const double& start_time, 
    const double& end_time,
    const double& time_step,
    std::vector<transform::TimedPose>& output_pose_queue,
    const bool& relative_to_end,
    const double& time_error) 
{
    UniqLock lock(interp_data_mutex_);
    if (interpolated_pose_queue_.empty()) {
        return false;
    }
    if (start_time >= end_time) {
        return false;
    }
    if (start_time < interpolated_pose_queue_.front().timestamp ||
        end_time > interpolated_pose_queue_.back().timestamp) {
        return false;
    }

    output_pose_queue.clear();

    /**
     * 每一个step: [0, step), pose取0时刻!
     * 输出应保证: start_time作为零时刻，属于第一个step；end_time应当被包含于最后一个step
     * 输出的效果: [start_time, start_time+step), ... , [xx_time, end_time, xx_time+step)
     * 
     * 对下方代码实现来说: current_time时刻的pose代表[current_time, current_time+step)时段的pose
    */

    auto it = interpolated_pose_queue_.begin();
    const auto it_end = interpolated_pose_queue_.end();
    double current_time = start_time;
    while (current_time <= end_time) {
        
        while (std::next(it) != it_end 
            && std::next(it)->timestamp < current_time) {
            ++it;
        }

        if (it->timestamp <= current_time && std::next(it) != it_end) {
            const double ratio = (current_time - it->timestamp) / 
                (std::next(it)->timestamp - it->timestamp);
            Eigen::Vector3d position = 
                it->pose.translation() * (1 - ratio) + 
                std::next(it)->pose.translation() * ratio;
            Eigen::Quaterniond orient = 
                it->pose.rotation().slerp(ratio, std::next(it)->pose.rotation());
            output_pose_queue.push_back(transform::TimedPose{
                common::Time(common::FromSeconds(current_time)), 
                current_time, transform::Rigid3d(position, orient)});
        }

        current_time += time_step;
    }

    const int step_counts = std::ceil((end_time - start_time) / time_step);
    if (output_pose_queue.size() != step_counts) {
        LOG(WARNING) << "error, output pose queue size wrong! (" 
            << output_pose_queue.size() << " vs " << step_counts << ").";
        return false;
    }
    return true;
}

// 支持多个连续时刻的内插查询
bool ImuAidedPoseInterpolator::LookUp(
    const std::vector<double>& interp_times, 
    std::vector<transform::TimedPose>& output_pose_queue,
    const bool& relative_to_end,
    const double& time_error) {
    UniqLock lock(interp_data_mutex_);
    // TODO
    return false;
}

std::deque<transform::TimedPose> 
ImuAidedPoseInterpolator::GetInternalPoseQueue() {
    UniqLock lock(interp_data_mutex_);
    return interpolated_pose_queue_;
}


// ***************************************************************************

void ImuAidedPoseInterpolator::Clear() {
    UniqLock lock(all_data_mutex_);
    UniqLock lock2(interp_data_mutex_);
    source_pose_queue_.clear();
    source_imu_queue_.clear();
    interpolated_pose_queue_.clear();
}

bool ImuAidedPoseInterpolator::Has(double timepoint) const {
    UniqLock lock(interp_data_mutex_);
    if (interpolated_pose_queue_.size() < 2) return false;
    return interpolated_pose_queue_.front().timestamp < timepoint 
            && interpolated_pose_queue_.back().timestamp > timepoint;
}

double ImuAidedPoseInterpolator::earliest_time() const {
    UniqLock lock(interp_data_mutex_);
    if (interpolated_pose_queue_.empty()) return -1.0;
    return interpolated_pose_queue_.front().timestamp;
}
double ImuAidedPoseInterpolator::latest_time() const {
    UniqLock lock(interp_data_mutex_);
    if (interpolated_pose_queue_.empty()) return -1.0;
    return interpolated_pose_queue_.back().timestamp;
}

bool ImuAidedPoseInterpolator::empty() const {
    UniqLock lock(interp_data_mutex_);
    return interpolated_pose_queue_.empty();
}
size_t ImuAidedPoseInterpolator::size() const {
    UniqLock lock(interp_data_mutex_);
    return interpolated_pose_queue_.size();
}

double ImuAidedPoseInterpolator::time_range_limit() const {
    UniqLock lock(all_data_mutex_);
    return kTimeRangeLimit;
    //
}
double ImuAidedPoseInterpolator::time_range() const {
    UniqLock lock(interp_data_mutex_);
    if (interpolated_pose_queue_.size() < 2) return 0.0;
    return interpolated_pose_queue_.back().timestamp 
            - interpolated_pose_queue_.front().timestamp;
}

// ***************************************************************************

void ImuAidedPoseInterpolator::AdvanceInterpolation() {
    // 初始化
    if (source_pose_queue_.empty()) return;
    if (interpolated_pose_queue_.empty()) {
        interpolated_pose_queue_.push_back(source_pose_queue_.front());
    }
    
    if (source_imu_queue_.empty()) {
        LOG(INFO) << "Waiting for imu ...";
        return;
    }
    if (source_imu_queue_.back().timestamp < source_pose_queue_.back().timestamp) {
        LOG(INFO) << "Waiting for imu to surpass pose ...";
        return;
    }
    const double source_common_time = 
        std::fmin(source_pose_queue_.back().timestamp, 
            source_imu_queue_.back().timestamp);
    const double last_interp_time = interpolated_pose_queue_.back().timestamp;

    // 时间有推进了才能advance
    if (source_common_time <= (last_interp_time)) return;
    double time_to_advance = source_common_time - last_interp_time;
    if (kDebugMode) {
        LOG(INFO) << "source common timestamp " << source_common_time 
            << ", leading " << time_to_advance << "s.";
    }

    // 找出所有可推进的工作单元
    /**
     * 找出所有可推进的工作单元
     * 已插值pose队列： --|----|----|
     * 原料的pose队列： -------|----|----|
     * 原料的imu 队列： ----||||||||||||||||
     * 相关的imu 范围： ------------||||||
     * 选中的imu 范围： ------------(----]    #时段上左开右闭#
    */
    std::vector<InterpTask> task_cache;
    double curr_interp_time = last_interp_time;
    auto curr_pose_it = source_pose_queue_.begin();
    auto curr_imu_it = source_imu_queue_.begin();
    for (auto it = source_pose_queue_.begin(); std::next(it) != source_pose_queue_.end(); ++it) {
        if (it->timestamp <= (curr_interp_time) 
            && std::next(it)->timestamp > (curr_interp_time)
            && std::next(it)->timestamp <= (source_common_time))
        {
            const double imu_st_time = it->timestamp;
            const double imu_ed_time = std::next(it)->timestamp;
            auto imu_st_it = curr_imu_it;
            auto imu_ed_it = curr_imu_it;
            for (auto imu_it = curr_imu_it; std::next(imu_it) != source_imu_queue_.end(); ++imu_it) {
                if (imu_it->timestamp <= imu_st_time
                    && std::next(imu_it)->timestamp > imu_st_time) {
                    imu_st_it = std::next(imu_it);
                }
                if (imu_it->timestamp <= imu_ed_time
                    && std::next(imu_it)->timestamp > imu_ed_time) {
                    imu_ed_it = imu_it;
                }
            }
            task_cache.emplace_back(InterpTask{*it, *std::next(it), imu_st_it, imu_ed_it});
            curr_imu_it = imu_ed_it;
            curr_pose_it = it;
        }
    }

    // 逐个执行工作单元
    for (auto task : task_cache) {
        AdvanceOneTask(task, interpolated_pose_queue_);
    }
    if (kDebugMode) {
        LOG(INFO) << "Finished " << task_cache.size() <<  " interpolation jobs, advanced "
            << source_pose_queue_.back().timestamp - last_interp_time << " seconds.";
    }

    // 丢掉用过了的source的数据。
    if (curr_pose_it != source_pose_queue_.begin()) {
        source_pose_queue_.erase(source_pose_queue_.begin(), curr_pose_it); //不包含curr_pose_it。
    }
    if (curr_imu_it != source_imu_queue_.begin()) {
        source_imu_queue_.erase(source_imu_queue_.begin(), curr_imu_it); //不包含curr_imu_it。
    }

}

void ImuAidedPoseInterpolator::AdvanceOneTask(
    const InterpTask& task,
    std::deque<transform::TimedPose>& target_pose_queue) 
{
    if (kDebugMode) {
        LOG(INFO) << "Handling task with pose duration " << std::fixed << std::setprecision(3)
            << task.next_pose.timestamp - task.curr_pose.timestamp << "s, and "
            << task.imu_ed_it - task.imu_st_it + 1 << " IMUs.";
    }

    // 平移按照匀速模型，旋转按照IMU角速度积分
    const double duration = task.next_pose.timestamp - task.curr_pose.timestamp;
    double current_time = task.curr_pose.timestamp;
    Eigen::Quaterniond current_rotation = task.curr_pose.pose.rotation();
    auto it = task.imu_st_it;
    bool stop = false;
    while (true) {
        const double ratio = (it->timestamp - task.curr_pose.timestamp) / duration;
        Eigen::Vector3d position = (1 - ratio) * task.curr_pose.pose.translation() 
                                    + ratio * task.next_pose.pose.translation();
        const double delta_t = it->timestamp - current_time;
        const Eigen::Vector3d delta_angle = it->angular_velocity * delta_t;
        Eigen::Quaterniond next_rotation = current_rotation *
            transform::AngleAxisVectorToRotationQuaternion(delta_angle);
        target_pose_queue.emplace_back(
            transform::TimedPose{it->time, it->timestamp, 
                transform::Rigid3d(position, next_rotation)});
        current_time = it->timestamp;
        current_rotation = next_rotation;
        if (it == task.imu_ed_it) {
            if (target_pose_queue.back().timestamp >= task.next_pose.timestamp) {
                target_pose_queue.pop_back();
            }
            target_pose_queue.push_back(task.next_pose);
            break;
        }
        ++it;
    }

    return;
}

void ImuAidedPoseInterpolator::TrimSourcePoseQueueIfNeeded() {
    while (source_pose_queue_.size() > 10 
        && (source_pose_queue_.back().timestamp
            - source_pose_queue_.front().timestamp)
                > kTimeRangeLimit) {
        source_pose_queue_.pop_front();
    }
}

void ImuAidedPoseInterpolator::TrimInterpPoseQueueIfNeeded() {
    while (interpolated_pose_queue_.size() > 10 
        && (interpolated_pose_queue_.back().timestamp
            - interpolated_pose_queue_.front().timestamp)
                > kTimeRangeLimit) {
        interpolated_pose_queue_.pop_front();
    }
}

// ***************************************************************************

bool InterpolatePoseQueueWithIMUs(  // template<typename PoseType>
    const double& start_time, 
    const double& end_time,
    const std::vector<transform::TimedPose>& source_pose_queue,
    const std::vector<sensor::ImuData>& source_imu_queue,
    const double& time_step,
    std::vector<transform::TimedPose>& output_pose_queue,
    const bool& relative_to_end,
    const double& time_error)
{
    output_pose_queue.clear();
    if (source_pose_queue.empty()) {
        LOG(ERROR) << "Interpolate pose queue failed: source pose queue is EMPTY!";
        return false;
    }
    if (start_time >= end_time ) { 
        LOG(ERROR) << "Interpolate pose queue failed: start_time BIGGER than end_time!";
        return false; 
    }
    if (start_time < (source_pose_queue.front().timestamp - time_error)
        || end_time > (source_pose_queue.back().timestamp + time_error)) {
        LOG(ERROR) << "Interpolate pose queue failed: time error too large!";
        return false; 
    }

    /**
     * 插值条件：
     * -- source pose 队列非空；
     * -- 开始～结束时间为顺序；
     * -- source pose 队列覆盖开始～结束时间段；
     * -- （IMU队列不强制要求，可为空或部分缺失）
     * 
     * 点云去畸变原理（时间对齐）：
     * time point: **************** t1 ********** t2 ********** tn *****
     * pose queue: **************** T1 ********** T2 ********** Tn *****
     * pose queue: **************** |  ********** |  ********** |  *****
     * coordinate of point 'P': *** p1 ********** p2 ********** pn *****
     * For point P sampled at t1 timepoint with coordinate p1, we try to
     * get the coordinate of P represented at t2 timepoint, i.e., p2.
     * Note that "T1, T2, ... Tn" are already given -- from interpolation!
     * we have T1 * p1 = T2 * p2, then p2 = T2^(-1) * T1 * p1,
     * so the element in output pose queue should be: {T_e^(-1) * T_i},
     * in which "T_e" means pose of end time.
    */

    const double duration = end_time - start_time;
    const int steps = std::ceil(duration / time_step) + 0.1;
    if (steps <= 0) {
        LOG(ERROR) << "Interpolate pose queue failed: "
            << "interp steps should be >= 1, which is " << steps;
        return false; 
    }
    output_pose_queue.reserve(steps);

    for (int i=0; i<steps; ++i) {
        //
    }

    // 首先得到绝对位姿插值表。
    transform::Rigid3d st_pose;
    transform::Rigid3d ed_pose;
    bool st_pose_found = false;
    bool ed_pose_found = false;
    for (auto& timed_pose : source_pose_queue) {
        if (timed_pose.timestamp == start_time) {
            st_pose = timed_pose.pose;
            st_pose_found = true;
        }
        if (timed_pose.timestamp == end_time) {
            ed_pose = timed_pose.pose;
            ed_pose_found = true;
        }
    }
    if (!st_pose_found) {
        for (int i=0; i<source_pose_queue.size()-1; ++i) {
            if (source_pose_queue[i].timestamp < start_time
                && source_pose_queue[i+1].timestamp > start_time) 
            {
                //
            }
        }
    }

    for (int i=0; i<steps; ++i) {
        //
    }



    // 如果指定了需要输出相对位姿，我们再做一次位姿转换。
    if (relative_to_end) {
        //
    }


    return true;
}



} // namespace utils
} // namespace infinityslam