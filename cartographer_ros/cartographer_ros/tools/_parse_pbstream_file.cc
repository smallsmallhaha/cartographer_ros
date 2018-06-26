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

#include "cartographer_ros/assets_writer.h"

#include <algorithm>
#include <fstream>
#include <iostream>

#include "cartographer/common/make_unique.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/time_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/time.h"

DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");

namespace carto = ::cartographer;
using std::cout;
using std::endl;

void OutputPoseToFile(std::ostream& out,
                      google::protobuf::RepeatedPtrField<
                          carto::mapping::proto::Trajectory_Node>& nodes) {
  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    carto::mapping::proto::Trajectory_Node& node = *it;
    carto::transform::proto::Vector3d trans = node.pose().translation();
    int64_t uts_timestamp = node.timestamp();
    int64_t ns_since_unix_epoch =
        (uts_timestamp -
         carto::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) *
        100ll;
    out << ns_since_unix_epoch << ", " << trans.x() << ", " << trans.y()
        << endl;
  }
}

void Run(std::string pose_graph_filename) {
  carto::mapping::proto::PoseGraph pose_graph(
      carto::io::DeserializePoseGraphFromFile(pose_graph_filename));

  const carto::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(pose_graph.trajectory(0));

  {
    cout << "pose_graph bytesize:  " << pose_graph.ByteSize() << endl;
    cout << "trajectory count:  " << pose_graph.trajectory().size() << endl;
    cout << "constraint count:  " << pose_graph.constraint().size() << endl;
  }

  {
    CHECK_GT(pose_graph.trajectory().size(), 0);

    auto nodes = pose_graph.trajectory(0).node();
    auto submaps = pose_graph.trajectory(0).submap();

    cout << "node count: " << nodes.size() << endl;
    cout << "submap count: " << submaps.size() << endl;

    std::ofstream out(pose_graph_filename + ".csv");

    OutputPoseToFile(out, nodes);
    //     OutputPoseToFile(cout, nodes);
  }
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";

  Run(FLAGS_pose_graph_filename);
}
