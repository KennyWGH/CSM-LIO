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

#ifndef INFINITYSLAM_CSMLIO_SUBMAP_3D_H_
#define INFINITYSLAM_CSMLIO_SUBMAP_3D_H_

#include <memory>
#include <string>
#include <vector>
#include "Eigen/Geometry"

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/sensor/range_data.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/transform.h"
#include "infinityslam/csmlio/submap/hybrid_grid.h"
#include "infinityslam/csmlio/submap/range_data_inserter_3d.h"
#include "infinityslam/csmlio/submap/submaps.h"
#include "infinityslam/csmlio/id.h"

namespace infinityslam {
namespace csmlio {

struct ActiveSubmaps3DOptions {
    double submap_high_resolution = 0.10;
    double submap_high_resolution_max_range = 20.;
    double submap_low_resolution = 0.30;
    int    submap_num_range_data = 100;
    double submap_insertion_hit_probability = 0.55;
    double submap_insertion_miss_probability = 0.49;
    int    submap_insertion_num_free_space_voxels = 2;
    double submap_insertion_intensity_threshold = 10.;

    ActiveSubmaps3DOptions() {}

    std::string DebugString() const {
        std::ostringstream oss;
        oss << "ActiveSubmaps3DOptions Info: \n";
        oss << "    submap_high_resolution           : " << submap_high_resolution << "\n"
            << "    submap_high_resolution_max_range : " << submap_high_resolution_max_range << "\n"
            << "    submap_low_resolution            : " << submap_low_resolution << "\n"
            << "    submap_num_range_data            : " << submap_num_range_data << "\n"
            << "    submap_insertion_hit_probability       : " << submap_insertion_hit_probability << "\n"
            << "    submap_insertion_miss_probability      : " << submap_insertion_miss_probability << "\n"
            << "    submap_insertion_num_free_space_voxels : " << submap_insertion_num_free_space_voxels << "\n"
            << "    submap_insertion_intensity_threshold   : " << submap_insertion_intensity_threshold;
        return oss.str();
    }
};

// 从common加载参数
ActiveSubmaps3DOptions ReadActiveSubmaps3DOptions();

class Submap3D : public Submap {
 public:
    Submap3D(float high_resolution, float low_resolution,
            const transform::Rigid3d& local_submap_pose,
            const Eigen::VectorXf& rotational_scan_matcher_histogram);

    // 外部UI可视化submap的接口
    bool ToPointCloud(const transform::Rigid3d& global_submap_pose, 
        const std::shared_ptr<sensor::PointCloud>& point_cloud_,
        bool from_high_res_submap = true) const;

    const HybridGrid& high_resolution_hybrid_grid() const {
        return *high_resolution_hybrid_grid_;
    }
    const HybridGrid& low_resolution_hybrid_grid() const {
        return *low_resolution_hybrid_grid_;
    }
    const IntensityHybridGrid& high_resolution_intensity_hybrid_grid() const {
        CHECK(high_resolution_intensity_hybrid_grid_ != nullptr);
        return *high_resolution_intensity_hybrid_grid_;
    }
    void ForgetIntensityHybridGrid() {
        high_resolution_intensity_hybrid_grid_.reset();
    }

    const Eigen::VectorXf& rotational_scan_matcher_histogram() const {
        return rotational_scan_matcher_histogram_;
    }

    // Insert 'range_data' into this submap using 'range_data_inserter'. The
    // submap must not be finished yet.
    void InsertData(const sensor::RangeData& range_data,
                    const RangeDataInserter3D& range_data_inserter,
                    float high_resolution_max_range,
                    const Eigen::Quaterniond& local_from_gravity_aligned,
                    const Eigen::VectorXf& scan_histogram_in_gravity);

    void Finish();

 private:
    // void UpdateFromProto(const proto::Submap3D& submap_3d);

    std::unique_ptr<HybridGrid> high_resolution_hybrid_grid_;
    std::unique_ptr<HybridGrid> low_resolution_hybrid_grid_;
    std::unique_ptr<IntensityHybridGrid> high_resolution_intensity_hybrid_grid_;
    Eigen::VectorXf rotational_scan_matcher_histogram_;
};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps3D {
 public:
    explicit ActiveSubmaps3D(const ActiveSubmaps3DOptions& options);

    ActiveSubmaps3D(const ActiveSubmaps3D&) = delete;
    ActiveSubmaps3D& operator=(const ActiveSubmaps3D&) = delete;

    // Inserts 'range_data_in_local' into the Submap collection.
    // 'local_from_gravity_aligned' is used for the orientation of new submaps so
    // that the z axis approximately aligns with gravity.
    // 'rotational_scan_matcher_histogram_in_gravity' will be accumulated in all
    // submaps of the Submap collection.
    std::vector<std::shared_ptr<const Submap3D>> InsertData(
        const sensor::RangeData& range_data_in_local,
        const Eigen::Quaterniond& local_from_gravity_aligned,
        const Eigen::VectorXf& rotational_scan_matcher_histogram_in_gravity);

    std::vector<std::shared_ptr<const Submap3D>> submaps() const;

 private:
    void AddSubmap(const transform::Rigid3d& local_submap_pose,
                    int rotational_scan_matcher_histogram_size);

    const ActiveSubmaps3DOptions full_Options_;
    std::vector<std::shared_ptr<Submap3D>> submaps_;
    RangeDataInserter3D range_data_inserter_;
};

// 从common参数构造ActiveSubmaps3D对象
std::unique_ptr<ActiveSubmaps3D> CreateActiveSubmaps3D();

}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_SUBMAP_3D_H_
