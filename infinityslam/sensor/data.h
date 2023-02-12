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
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_CSMLIO_DATA_H_
#define INFINITYSLAM_CSMLIO_DATA_H_

#include <boost/make_unique.hpp>
#include "glog/logging.h"
#include "infinityslam/common/time.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {

namespace csmlio {
class CSMLioInterface;
}

namespace sensor {

class Data {
  public:
    explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
    virtual ~Data() {}

    virtual common::Time GetTime() const = 0;
    const std::string &GetSensorId() const { return sensor_id_; }
    virtual void AddToLIO(csmlio::CSMLioInterface* lio_owner) = 0;

  protected:
    const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_DATA_H_
