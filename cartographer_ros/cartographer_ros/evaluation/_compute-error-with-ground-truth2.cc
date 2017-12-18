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
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/split_string.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "ceres/ceres.h"
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

#include "cartographer_ros/evaluation/transform-all-coord-into-one-system.hpp"

DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");

using namespace ::cartographer_ros;
namespace carto = ::cartographer;
using std::cout;
using std::endl;
using std::vector;

// 价值函数
class CostFunctor {
public:
  CostFunctor(vector<vector<double>> &curve, vector<vector<double>> &points)
      : curve_(curve), points_(points) {}

  CostFunctor(const CostFunctor &) = delete;
  CostFunctor &operator=(const CostFunctor &) = delete;

  bool operator()(const double *const transform, double *residual) const {
    for (std::size_t i = 0; i < points_.size(); ++i) {
      double dx = transform[0];
      double dy = transform[1];
      double s = ceres::sin(transform[2]);
      double c = ceres::cos(transform[2]);
      double x1 = points_[i][0];
      double y1 = points_[i][1];
      double x2 = x1 * c + y1 * s + dx;
      double y2 = y1 * c - x1 * s + dy;

      double mindist2 = (x2 - curve_[0][0]) * (x2 - curve_[0][0]) +
                        (y2 - curve_[0][1]) * (y2 - curve_[0][1]);
      for (std::size_t j = 0; j < curve_.size(); ++j) {
        double newdist2 = (x2 - curve_[j][0]) * (x2 - curve_[j][0]) +
                          (y2 - curve_[j][1]) * (y2 - curve_[j][1]);
        mindist2 = mindist2 < newdist2 ? mindist2 : newdist2;
      }

      residual[i] = sqrt(mindist2);
    }
    return true;
  }

private:
  vector<vector<double>> curve_;
  vector<vector<double>> points_;
};

// 获取要拟合的曲线
void GetCurve(const string &pose_graph_filename,
              vector<vector<double>> &curve) {
  carto::io::ProtoStreamReader reader("/home/whu/Documents/data/17F/"
                                      "origin_2017-12-15-16-04-13.out.bag."
                                      "pbstream");
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  reader.ReadProto(&pose_graph_proto);

  const carto::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(pose_graph_proto.trajectory(0));

  //   cout << "pose_graph_proto bytesize:  " << pose_graph_proto.ByteSize() <<
  //   endl;
  //   cout << "trajectory count:  " << pose_graph_proto.trajectory().size() <<
  //   endl;
  //   cout << "constraint count:  " << pose_graph_proto.constraint().size() <<
  //   endl;

  {
    auto nodes = pose_graph_proto.trajectory(0).node();
    auto submaps = pose_graph_proto.trajectory(0).submap();

    //     cout << "node count: " << nodes.size() << endl;
    //     cout << "submap count: " << submaps.size() << endl;

    for (const carto::mapping::proto::Trajectory::Node &node :
         pose_graph_proto.trajectory(0).node()) {
      const ::carto::transform::Rigid3d pose =
          carto::transform::ToRigid3(node.pose());
      Eigen::Matrix<double, 3, 1> translation = pose.translation();
      //       cout << translation;
      curve.push_back({translation(0, 0), translation(1, 0)});
      //       cout << "[" << translation(0, 0) << "," << translation(1, 0) <<
      //       "],"
      //            << endl;
    }
  }

  cartographer::common::Time time;
  if (transform_interpolation_buffer.Has(time)) {
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time);
    std::cout << "time in" << std::endl;
  }
}

void GetPoints(vector<vector<double>> &points) {}

void DisplayPoints(vector<vector<double>> &points, double *transform) {
  for (auto point : points) {
    double dx = transform[0];
    double dy = transform[1];
    double s = ceres::sin(transform[2]);
    double c = ceres::cos(transform[2]);
    double x1 = point[0];
    double y1 = point[1];
    double x2 = x1 * c + y1 * s + dx;
    double y2 = y1 * c - x1 * s + dy;
    //     cout << "[" << x2 << "," << y2 << "]," << endl;
  }
}

void Run(const string &pose_graph_filename, const string &bag_filenames,
         PointsPtr &points_ptr) {

  double transform[3] = {34.7, 3.16, 1.57};
  vector<vector<double>> curve;
  vector<vector<double>> points;
  for (auto it = points_ptr->begin(); it != points_ptr->end(); ++it) {
    points.push_back({it->x, it->y});
  };

  // 获取位姿曲线
  GetCurve(pose_graph_filename, curve);
  // 获取使用测量机器人测到的一些点
  GetPoints(points);

  // 使用 ceres 计算标准路径点坐标到位姿曲线的拟合误差
  ceres::Problem problem;

  problem.AddResidualBlock(
      // 分别表示: 残差方程形参 输入维数 输出维数
      new ceres::NumericDiffCostFunction<CostFunctor,
                                         ceres::NumericDiffMethodType::CENTRAL,
                                         ceres::DYNAMIC, 3>(
          // 参数为残差方程实参和残差个数
          new CostFunctor(curve, points), ceres::Ownership::TAKE_OWNERSHIP,
          points.size()),
      nullptr, transform);

  ceres::Solver::Options options;
  //   options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  //   cout << summary.FullReport() << endl;
  cout << summary.BriefReport() << endl;
  cout << transform[0] << " " << transform[1] << " " << transform[2] << endl;

  //   DisplayPoints(points, transform);
}

int main(int argc, char **argv) {

  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  //   CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  //   CHECK(!FLAGS_pose_graph_filename.empty())
  //       << "-pose_graph_filename is missing.";

  //   PointsPtr points_ptr =
  //   load_all_json_files_and_convert_into_one_coord_system(
  //       {"/home/whu/Documents/data/17F/1.csv.json",
  //        "/home/whu/Documents/data/17F/2.csv.json"});

  PointsPtr points_ptr = load_all_json_files_and_convert_into_one_coord_system(
      {"/home/whu/Documents/data/17F/1.csv.json"});

  //   cout << points_ptr << endl;

  Run(FLAGS_pose_graph_filename, FLAGS_bag_filenames, points_ptr);
}
