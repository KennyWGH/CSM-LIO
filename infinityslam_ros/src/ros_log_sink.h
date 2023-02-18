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

#ifndef ROS_APP_ROS_LOG_SINK_H
#define ROS_APP_ROS_LOG_SINK_H

#include <ctime>

#include "glog/logging.h"

namespace infinityslam_ros {

// 冠华，解释一下这个类是干嘛的？
// Makes Google logging use ROS logging for output while an instance of this
// class exists.
class ScopedRosLogSink : public ::google::LogSink {
 public:
  ScopedRosLogSink();
  ~ScopedRosLogSink() override;

  void send(::google::LogSeverity severity, const char* filename,
            const char* base_filename, int line, const struct std::tm* tm_time,
            const char* message, size_t message_len) override;

  void WaitTillSent() override;

 private:
  bool will_die_;
};

}  // namespace infinityslam_ros

#endif  // ROS_APP_ROS_LOG_SINK_H
