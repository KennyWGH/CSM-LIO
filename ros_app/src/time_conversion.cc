/**
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include "ros_app/src/time_conversion.h"

#include "infinityslam/common/time.h"
#include "ros/ros.h"

namespace ros_app {

::ros::Time ToRos(::infinityslam::common::Time time) {
    int64_t uts_timestamp = ::infinityslam::common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
        (uts_timestamp -
        ::infinityslam::common::kUtsEpochOffsetFromUnixEpochInSeconds *
            10000000ll) *
        100ll;
    ::ros::Time ros_time;
    ros_time.fromNSec(ns_since_unix_epoch);
    return ros_time;
}

// TODO(pedrofernandez): Write test.
::infinityslam::common::Time FromRos(const ::ros::Time& time) {
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return ::infinityslam::common::FromUniversal(
        (time.sec +
        ::infinityslam::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
        (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace ros_app
