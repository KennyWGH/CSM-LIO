/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_COMMON_FIXED_RATIO_SAMPLER_H_
#define INFINITYSLAM_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "infinityslam/common/numeric_types.h"

namespace infinityslam {
namespace common {

// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
class FixedRatioSampler {
 public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  bool Pulse();

  // Returns a debug string describing the current ratio of samples to pulses.
  std::string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  const double ratio_;

  int64 num_pulses_ = 0;
  int64 num_samples_ = 0;
};

}  // namespace common
}  // namespace infinityslam

#endif  // INFINITYSLAM_COMMON_FIXED_RATIO_SAMPLER_H_
