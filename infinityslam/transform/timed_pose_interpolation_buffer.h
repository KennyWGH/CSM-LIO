/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_TRANSFORM_TIMED_POSE_INTERPOLATION_BUFFER_H_
#define INFINITYSLAM_TRANSFORM_TIMED_POSE_INTERPOLATION_BUFFER_H_

#include <deque>

#include "infinityslam/transform/timed_pose.h"

namespace infinityslam {
namespace transform {


/**
 * 在 TimedPose 队列上做内插外推。
 * 注意【外推】应当是有时间限制的。
 * 我们特意用【double】类型的时间表示。
*/
class TimedPoseInterpolationBuffer {
 public:
  TimedPoseInterpolationBuffer() = default;

  // 最多保存多长时间的位姿队列。
  void SetTimeRangeLimit(double max_time_range);

  // 在队尾新增位姿，并在需要时移除最早的。
  void Push(double time, const transform::TimedPose& transform);

  // 清空队列.
  void Clear();

  // 时间是否在队列时间范围内 —— 也即能否内插。
  bool Has(double time) const;

  // 内插位姿。
  transform::TimedPose Lookup(double time) const;

  double earliest_time() const;
  double latest_time() const;

  bool empty() const;

  double time_range_limit() const;
  double time_range() const;

  size_t size() const;

  

 private:
  void RemoveOldPosesIfNeeded();

  std::deque<TimedPose> timed_pose_cache_;
  double time_range_limit_ = 10.;
};









}  // namespace transform
}  // namespace infinityslam

#endif  // INFINITYSLAM_TRANSFORM_TIMED_POSE_INTERPOLATION_BUFFER_H_
