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

#ifndef CSMLIO_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CSMLIO_SENSOR_INTERNAL_DISPATCHABLE_H_

// #include "csmlio/lio/trajectory_builder_interface.h"
#include "csmlio/sensor/data.h"
#include "csmlio/lio/csm_lio_interface.h" 
// #include "csmlio/lio/csm_lidar_inertial_odometry.h" //交叉包含问题，编译通不过。

namespace csmlio {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
  public:
    Dispatchable(const std::string &sensor_id, const DataType &data)
        : Data(sensor_id), data_(data) {}

    common::Time GetTime() const override { return data_.time; }
    void AddToLIO(mapping::CSMLioInterface *const lio_owner) override {
        lio_owner->ProcessSensorData(sensor_id_, data_);
    }
    const DataType &data() const { return data_; }

  private:
    const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
    return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_SENSOR_INTERNAL_DISPATCHABLE_H_
