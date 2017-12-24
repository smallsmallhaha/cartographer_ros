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
#include "cartographer/common/time.h"
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

DEFINE_string(point_filenames, "", "Points data with timestamp infomation "
                                   "measured by Measurement Robot(Leica TS60)"
                                   "in json format.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");

using namespace ::cartographer_ros;
namespace carto = ::cartographer;
using std::cout;
using std::endl;
using std::vector;

// 价值函数，计算包含时间偏移的坐标转换
class CostFunctor {
public:
  CostFunctor(vector<vector<double>> &curve, const PointsPtr &points,
              const carto::transform::TransformInterpolationBuffer
                  &transform_interpolation_buffer)
      : curve_(curve), points_(points),
        transform_interpolation_buffer_(transform_interpolation_buffer) {
    CHECK_GT(points_->size(), 0);

    std::size_t earliest_point_index_ = 0;
    std::size_t latest_point_index_ = 0;
    for (std::size_t i = 1; i < points_->size(); ++i) {
      Point &point = (*points_)[i];
      if (point.timestamp < (*points_)[earliest_point_index_].timestamp)
        earliest_point_index_ = i;
      if (point.timestamp > (*points_)[latest_point_index_].timestamp)
        latest_point_index_ = i;
    }

    cout << earliest_point_index_ << " " << latest_point_index_ << endl;

    cout << transform_interpolation_buffer.earliest_time() << " "
         << transform_interpolation_buffer.latest_time() << endl;

    cout << (*points_)[earliest_point_index_] << " "
         << (*points_)[latest_point_index_] << endl;

    min_delta_t =
        carto::common::ToUniversal(
            transform_interpolation_buffer_.earliest_time()) -
        carto::common::ToUniversal((*points_)[earliest_point_index_].timestamp);
    max_delta_t =
        carto::common::ToUniversal(
            transform_interpolation_buffer_.latest_time()) -
        carto::common::ToUniversal((*points_)[latest_point_index_].timestamp);

    CHECK_GE(max_delta_t, min_delta_t);
  }

  CostFunctor(const CostFunctor &) = delete;
  CostFunctor &operator=(const CostFunctor &) = delete;

  bool operator()(const double *const transform, double *residual) const {
    double dx = transform[0];
    double dy = transform[1];
    double s = sin(transform[2]);
    double c = cos(transform[2]);
    int64 delta_t = static_cast<int64>(transform[3]);
    delta_t = carto::common::Clamp(delta_t, min_delta_t, max_delta_t);
    // to do
    //     (*(double **)transform)[3] = delta_t;  // program crashed
    //     cout << transform[3] << " " << delta_t << endl;
    for (std::size_t i = 0; i < points_->size(); ++i) {
      Point &point = (*points_)[i];
      double x1 = point.x;
      double y1 = point.y;
      auto vector_xy = GetPose(point.timestamp, delta_t);
      double x2 = vector_xy[0];
      double y2 = vector_xy[1];
      residual[2 * i + 0] = x1 * c + y1 * s + dx - x2;
      residual[2 * i + 1] = y1 * c - x1 * s + dy - y2;
    }
    return true;
  }

private:
  vector<double> GetPose(const carto::common::Time &t,
                         const int64 &delta_t) const {
    carto::common::Time time =
        carto::common::FromUniversal(carto::common::ToUniversal(t) + delta_t);
    CHECK_EQ(transform_interpolation_buffer_.Has(time), true);
    carto::transform::Rigid3d pose =
        transform_interpolation_buffer_.Lookup(time);
    auto translation = pose.translation();
    return {translation[0], translation[1]};
  }

  int64 max_delta_t;
  int64 min_delta_t;
  vector<vector<double>> curve_;
  const PointsPtr &points_;
  const carto::transform::TransformInterpolationBuffer
      &transform_interpolation_buffer_;
};

void GetCurve(const string &pose_graph_filename, vector<vector<double>> &curve);

void Run(const string &pose_graph_filename, const string &bag_filenames,
         const std::vector<string> &point_filenames) {

  carto::io::ProtoStreamReader reader(pose_graph_filename);
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  reader.ReadProto(&pose_graph_proto);

  const carto::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(pose_graph_proto.trajectory(0));

  PointsPtr points_ptr =
      LoadAllJsonFilesAndConvertIntoOneSystem(point_filenames);

  double transform[] = {34.7, 3.16, 1.57, 0};

  vector<vector<double>> curve;
  // 获取位姿曲线
  GetCurve(pose_graph_filename, curve);

  // 使用 ceres 计算标准路径点坐标到位姿曲线的拟合误差
  ceres::Problem problem;

  problem.AddResidualBlock(
      // 分别表示: 残差方程形参 输入维数 输出维数
      new ceres::NumericDiffCostFunction<CostFunctor,
                                         ceres::NumericDiffMethodType::CENTRAL,
                                         ceres::DYNAMIC, 4>(
          // 参数为残差方程实参和残差个数
          new CostFunctor(curve, points_ptr, transform_interpolation_buffer),
          ceres::Ownership::TAKE_OWNERSHIP, points_ptr->size() * 2),
      nullptr, transform);

  ceres::Solver::Options options;
  //   options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  //   cout << summary.FullReport() << endl;
  cout << summary.BriefReport() << endl;
  cout << transform[0] << " " << transform[1] << " " << transform[2] << endl;
}

int main(int argc, char **argv) {

  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  //     CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";
  CHECK(!FLAGS_point_filenames.empty()) << "-point_filenames is missing.";

  Run(FLAGS_pose_graph_filename, FLAGS_bag_filenames,
      cartographer_ros::SplitString(FLAGS_point_filenames, ','));
}

// 获取要拟合的曲线
void GetCurve(const string &pose_graph_filename,
              vector<vector<double>> &curve) {
  carto::io::ProtoStreamReader reader(pose_graph_filename);
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  reader.ReadProto(&pose_graph_proto);

  const carto::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(pose_graph_proto.trajectory(0));

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
    }
  }

  cartographer::common::Time time;
  if (transform_interpolation_buffer.Has(time)) {
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time);
    std::cout << "time in" << std::endl;
  }
}
