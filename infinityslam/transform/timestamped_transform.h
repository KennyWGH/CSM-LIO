/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define INFINITYSLAM_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "infinityslam/common/time.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {
namespace transform {

struct TimestampedTransform {
    common::Time time;
    transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace transform
}  // namespace infinityslam

#endif  // INFINITYSLAM_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
