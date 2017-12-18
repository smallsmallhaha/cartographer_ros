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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/split_string.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"

DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");

using namespace std;
using namespace ::cartographer::mapping::proto;
using namespace cartographer_ros;
namespace carto = ::cartographer;

void Run(const string &pose_graph_filename, const string &bag_filenames) {
  // read ROS message from .bag file
  //   rosbag::Bag bag;
  //   bag.open(bag_filenames, rosbag::bagmode::Read);
  //   rosbag::View view(bag);
  //   const ::ros::Time begin_time = view.getBeginTime();
  //   const ::ros::Time end_time = view.getEndTime();
  //
  //   int count = 0;
  //
  //   std::cout << begin_time.toNSec() << " " << end_time.toNSec() << endl;
  //   std::cout << "time of .bag: " << (end_time-begin_time).toSec() << endl;
  //   for (const rosbag::MessageInstance& message : view) {
  //
  //       if(message.getTopic().compare("/horizontal_laser_2d")==0)
  //       {
  //           ++count;
  //
  //           auto
  //           laser_scan_message=*message.instantiate<sensor_msgs::LaserScan>();
  //
  //           const carto::common::Time start_time =
  //           FromRos(laser_scan_message.header.stamp);
  //
  //           auto points_batch =
  //           carto::common::make_unique<carto::io::PointsBatch>();
  //           points_batch->start_time = start_time;
  //           points_batch->frame_id = laser_scan_message.header.frame_id;
  //
  //           carto::sensor::PointCloudWithIntensities point_cloud =
  //               ToPointCloudWithIntensities(laser_scan_message);
  //           CHECK_EQ(point_cloud.intensities.size(),
  //           point_cloud.points.size());
  //           CHECK_EQ(point_cloud.offset_seconds.size(),
  //           point_cloud.points.size());
  //
  //       }
  //
  //   }
  /*
    std::cout << "horizontal_laser_2d number: " << count << std::endl;
    bag.close();*/

  // read sparse_pose_graph from .pbstream file
  carto::io::ProtoStreamReader reader(
      "/home/whu/Documents/backup/data/lidar_slam/loop-succeed-2/"
      "origin_2017-12-03-16-50-31.bag.pbstream");
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  reader.ReadProto(&pose_graph_proto);

  const carto::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(pose_graph_proto.trajectory(0));

  cout << "pose_graph_proto bytesize:  " << pose_graph_proto.ByteSize() << endl;
  cout << "trajectory count:  " << pose_graph_proto.trajectory().size() << endl;
  cout << "constraint count:  " << pose_graph_proto.constraint().size() << endl;

  {
    auto node = pose_graph_proto.trajectory(0).node();
    auto submap = pose_graph_proto.trajectory(0).submap();

    cout << "node count: " << node.size() << endl;
    cout << "submap count: " << submap.size() << endl;
  }

  cartographer::common::Time time;
  if (transform_interpolation_buffer.Has(time)) {
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time);
    std::cout << "time in" << std::endl;
  }

  for (auto &f : pose_graph_proto.constraint()) {
    carto::mapping::proto::SubmapId submap_id = f.submap_id();
    carto::mapping::proto::NodeId node_id = f.node_id();
    carto::transform::proto::Rigid3d relative_pose = f.relative_pose();
    double translation_weight = f.translation_weight();
    double rotation_weight = f.rotation_weight();
    carto::mapping::proto::SparsePoseGraph_Constraint_Tag tag = f.tag();

    cout << "CONSTRAINT:" << endl;
    cout << "submap_id:  " << submap_id.trajectory_id() << ","
         << submap_id.submap_index() << endl;
    cout << "node_id:  " << node_id.trajectory_id() << ","
         << node_id.node_index() << endl;
    cout << "relative_pose:" << relative_pose.DebugString() << endl;
    cout << "translation_weight:  " << translation_weight << endl;
    cout << "rotation_weight:  " << rotation_weight << endl;
    cout << "tag:  " << tag << endl;
  }
}

int main(int argc, char **argv) {
  //   FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  //   CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  //   CHECK(!FLAGS_pose_graph_filename.empty())
  //       << "-pose_graph_filename is missing.";

  Run(FLAGS_pose_graph_filename, FLAGS_bag_filenames);

  getchar();
}
