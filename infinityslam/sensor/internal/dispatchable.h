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

#ifndef INFINITYSLAM_SENSOR_INTERNAL_DISPATCHABLE_H_
#define INFINITYSLAM_SENSOR_INTERNAL_DISPATCHABLE_H_

// #include "infinityslam/csmlio/trajectory_builder_ interface.h"
#include "infinityslam/sensor/data.h"
#include "infinityslam/csmlio/csm_lio_interface.h" 
// #include "infinityslam/csmlio/csm_lidar_ine rtial_odometry.h" //交叉包含问题，编译通不过。

namespace infinityslam {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
  public:
    Dispatchable(const std::string &sensor_id, const DataType &data)
        : Data(sensor_id), data_(data) {}

    common::Time GetTime() const override { return data_.time; }
    void AddToLIO(csmlio::CSMLioInterface *const lio_owner) override {
        lio_owner->ProcessSensorData(sensor_id_, data_);
    }
    const DataType &data() const { return data_; }

  private:
    const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
    return boost::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_INTERNAL_DISPATCHABLE_H_
