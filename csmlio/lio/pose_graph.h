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

#ifndef CSMLIO_MAPPING_POSE_GRAPH_H_
#define CSMLIO_MAPPING_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/lio/id.h"
#include "csmlio/lio/pose_graph_interface.h"
// #include "csmlio/lio/pose_graph_ trimmer.h"
#include "csmlio/lio/proto/pose_graph.pb.h"
#include "csmlio/lio/proto/pose_graph_options.pb.h"
#include "csmlio/lio/proto/serialization.pb.h"
#include "csmlio/lio/submaps.h"
#include "csmlio/lio/trajectory_node.h"
#include "csmlio/sensor/fixed_frame_pose_data.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/landmark_data.h"
#include "csmlio/sensor/map_by_time.h"
#include "csmlio/sensor/odometry_data.h"

namespace csmlio {
namespace mapping {

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class PoseGraph : public PoseGraphInterface {
 public:
  struct InitialTrajectoryPose {
    int to_trajectory_id;
    transform::Rigid3d relative_pose;
    common::Time time;
  };

  PoseGraph() {}
  ~PoseGraph() override {}

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Inserts an IMU measurement.
  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;

  // Inserts an odometry measurement.
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::OdometryData& odometry_data) = 0;

  // Inserts a fixed frame pose measurement.
  virtual void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) = 0;

  // Inserts landmarks observations.
  virtual void AddLandmarkData(int trajectory_id,
                               const sensor::LandmarkData& landmark_data) = 0;

  // Finishes the given trajectory.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  virtual void FreezeTrajectory(int trajectory_id) = 0;

  // Adds a 'submap' from a proto with the given 'global_pose' to the
  // appropriate trajectory.
  virtual void AddSubmapFromProto(const transform::Rigid3d& global_pose,
                                  const proto::Submap& submap) = 0;

  // Adds a 'node' from a proto with the given 'global_pose' to the
  // appropriate trajectory.
  virtual void AddNodeFromProto(const transform::Rigid3d& global_pose,
                                const proto::Node& node) = 0;

  // Sets the trajectory data from a proto.
  virtual void SetTrajectoryDataFromProto(
      const mapping::proto::TrajectoryData& data) = 0;

  // Adds information that 'node_id' was inserted into 'submap_id'. The submap
  // has to be deserialized first.
  virtual void AddNodeToSubmap(const NodeId& node_id,
                               const SubmapId& submap_id) = 0;

  // Adds serialized constraints. The corresponding trajectory nodes and submaps
  // have to be deserialized before calling this function.
  virtual void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
//   virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Gets the current trajectory clusters.
  virtual std::vector<std::vector<int>> GetConnectedTrajectories() const = 0;

  proto::PoseGraph ToProto(bool include_unfinished_submaps) const override;

  // Returns the IMU data.
  virtual sensor::MapByTime<sensor::ImuData> GetImuData() const = 0;

  // Returns the odometry data.
  virtual sensor::MapByTime<sensor::OdometryData> GetOdometryData() const = 0;

  // Returns the fixed frame pose data.
  virtual sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
      const = 0;

  // Returns the landmark data.
  virtual std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  GetLandmarkNodes() const = 0;

  // Sets a relative initial pose 'relative_pose' for 'from_trajectory_id' with
  // respect to 'to_trajectory_id' at time 'time'.
  virtual void SetInitialTrajectoryPose(int from_trajectory_id,
                                        int to_trajectory_id,
                                        const transform::Rigid3d& pose,
                                        const common::Time time) = 0;
};

// std::vector<PoseGraph::Constraint> FromProto(
//     const ::google::protobuf::RepeatedPtrField<
//         ::csmlio::mapping::proto::PoseGraph::Constraint>&
//         constraint_protos);
proto::PoseGraph::Constraint ToProto(const PoseGraph::Constraint& constraint);

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_POSE_GRAPH_H_
