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

/**
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef CSMLIO_MAPPING_DATA_H_
#define CSMLIO_MAPPING_DATA_H_

#include "absl/memory/memory.h"
#include "glog/logging.h"
#include "csmlio/common/time.h"
#include "csmlio/transform/rigid_transform.h"

namespace csmlio {

namespace mapping {
class CSMLioInterface;
}

namespace sensor {

class Data {
  public:
    explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
    virtual ~Data() {}

    virtual common::Time GetTime() const = 0;
    const std::string &GetSensorId() const { return sensor_id_; }
    // virtual void AddToTrajectoryBuilder(
    //     mapping::TrajectoryBuilderInterface *trajectory_builder) = 0;
    virtual void AddToLIO(mapping::CSMLioInterface* lio_owner) = 0;

  protected:
    const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_DATA_H_
