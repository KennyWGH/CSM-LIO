/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INFINITYSLAM_COMMON_TIME_H_
#define INFINITYSLAM_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "infinityslam/common/numeric_types.h"

namespace infinityslam {
namespace common {

/**
 * 我们区分几个时间概念：
 * -- 1. Universal Time Scale, 世界时/or格林威治时间；
 * -- 2. Universal Time Coordinated, UTC时间/or协调世界时，以原子时秒长为基础，
 *       在时刻上尽量接近于世界时的一种时间计量系统；
 * -- 3. Unix时间，从协调世界时1970年1月1日0时0分0秒起至现在的总秒数，
 *       也即以UTC"1970-01-01 00:00:00.0 +0000"为零时刻，以原子钟秒长作为计秒单位；
 * -- 4. Universal时间，以UTC"0001-01-01 00:00:00.0 +0000"起至现在的总秒数；
 * 
 * -- 一、在本项目中，为了避免时间出现负数，我们统一使用Universal时间；
 * -- 二、大多数驱动给出的数据时刻都取计算机时刻，也即Unix时间；（包括ROS吗）
 * -- 三、Universal时间与Unix时间，两者相差719162天，约6.21356e10秒。
*/
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll); 

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>; //【1e-7秒，0.1微妙，100纳秒】为计时单位
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(Duration duration);
double ToSeconds(std::chrono::steady_clock::duration duration);

// Returns the given time point in seconds.
double ToSeconds(Time time_point);

// Creates a time from a Universal Time Scale.
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
double GetThreadCpuTimeSeconds();

}  // namespace common
}  // namespace infinityslam

#endif  // INFINITYSLAM_COMMON_TIME_H_
