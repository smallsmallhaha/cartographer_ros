/**
 * 本文件将从多个文件中读取到的点坐标数据变换到一个公共坐标系下
 * 
 */

#ifndef TRANSFORM_INTO_ONE_COORD_SYSTEM_HPP
#define TRANSFORM_INTO_ONE_COORD_SYSTEM_HPP

#include "cartographer_ros/evaluation/parse_json_file.hpp"
#include "ceres/ceres.h"
#include <cmath>

using std::cout;
using std::endl;
using std::vector;

// 对应点，两个不同坐标系下的同一控制点坐标是一对对应点，用于坐标变换
struct CorrespondingPoints {
  Point *p1;
  Point *p2;
};

// 坐标转换价值函数
class TransformCostFunctor {
public:
  TransformCostFunctor(vector<vector<double>> data) : data_(data) {}

  TransformCostFunctor(const TransformCostFunctor &) = delete;
  TransformCostFunctor &operator=(const TransformCostFunctor &) = delete;

  template <typename T>
  bool operator()(const T *const trans, T *residual) const {
    for (int i = 0; i < data_.size(); ++i) {
      T dx = T(trans[0]);
      T dy = T(trans[1]);
      T s = ceres::sin(trans[2]);
      T c = ceres::cos(trans[2]);
      T x1 = T(data_[i][0]);
      T y1 = T(data_[i][1]);
      T x2 = T(data_[i][2]);
      T y2 = T(data_[i][3]);
      residual[2 * i + 0] = x1 * c + y1 * s + dx - x2;
      residual[2 * i + 1] = y1 * c - x1 * s + dy - y2;
    }
    return true;
  }

private:
  vector<vector<double>> data_;
};

// 获取两个坐标系下的公共点坐标
vector<CorrespondingPoints> GetCommonControlPoints(PointsPtr &pts1,
                                                   PointsPtr &pts2) {
  vector<CorrespondingPoints> cpts;
  // use control_point in pts1 to fill cpts
  for (auto it = pts1.get()->begin(); it != pts1.get()->end(); ++it) {
    if (it->is_control_point) {
      cpts.push_back({&*it, nullptr});
    }
  }
  // use control_point in pts2 to fill cpts
  for (auto it = pts2.get()->begin(); it != pts2.get()->end(); ++it) {
    if (it->is_control_point) {
      for (auto it_ = cpts.begin(); it_ != cpts.end(); ++it_) {
        auto s = it_->p1;
        auto s1 = s->name;
        if (std::strcmp(it_->p1->name.c_str(), it->name.c_str()) == 0)
          it_->p2 = &*it;
      }
    }
  }
  // remove element of cpts who has only one control point
  for (auto it = cpts.begin(); it != cpts.end();) {
    if (it->p2 == nullptr)
      it = cpts.erase(it);
    else
      ++it;
  }
  return cpts;
}

// 计算两个坐标系下的转换，用到了ceres
vector<double> ComputeTransformBetweenTwoStation(PointsPtr &pts1,
                                                 PointsPtr &pts2) {
  auto cpts = GetCommonControlPoints(pts1, pts2);

  if (cpts.size() < 2) {
    LOG(WARNING) << "common control points too few, transform failed";
    return vector<double>();
  }

  ceres::Problem problem;

  double transform[3] = {0};
  vector<vector<double>> data;
  for (auto it = cpts.begin(); it != cpts.end(); ++it)
    data.push_back({it->p1->x, it->p1->y, it->p2->x, it->p2->y});

  problem.AddResidualBlock(
      // 分别表示: 残差方程形参 输入维数 输出维数
      new ceres::AutoDiffCostFunction<TransformCostFunctor, ceres::DYNAMIC, 3>(
          // 参数为残差方程实参和残差(方程式)个数
          new TransformCostFunctor(data), data.size() * 2),
      nullptr, transform);

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  cout << summary.BriefReport() << endl;

  return {transform[0], transform[1], transform[2]};
}

// 加载filenames中所有文件的坐标数据，返回它们在同一坐标系下的坐标数据（不含控制点）
PointsPtr
LoadAllJsonFilesAndConvertIntoOneSystem(vector<std::string> filenames) {
  vector<PointsPtr> all_points;
  // load all measurement data
  for (auto it = filenames.begin(); it != filenames.end();) {
    PointsPtr points_ptr = ParseJsonFile(*it);
    // if parse file error, delete it from filenames
    if (points_ptr != nullptr) {
      LOG(INFO) << "load file \"" << *it << "\" success";
      all_points.push_back(points_ptr);
      ++it;
    } else {
      LOG(ERROR) << "load file \"" << *it << "\" failed";
      it = filenames.erase(it);
    }
  }

  // transform into one system
  PointsPtr trajectory_points_in_one_system(new Points);
  if (all_points.size() == 0) {
    LOG(ERROR) << "0 point data file found, exit.";
    exit(1);
  } else if (all_points.size() == 1) {
    std::for_each(all_points[0]->begin(), all_points[0]->end(),
                  [&trajectory_points_in_one_system](Point &p) {
                    if (!p.is_control_point)
                      trajectory_points_in_one_system->push_back(p);
                  });
    return trajectory_points_in_one_system;
  } else if (all_points.size() == 2) {
    for (std::size_t i = 0; i < all_points.size() - 1; ++i) {

      // insert all_points[i] into trajectory_points_in_one_system
      for (auto it = all_points[i].get()->begin();
           it != all_points[i].get()->end(); ++it) {
        if (!it->is_control_point) {
          trajectory_points_in_one_system->push_back(*it);
        }
      }

      auto transform =
          ComputeTransformBetweenTwoStation(all_points[i], all_points[i + 1]);

      double dx = transform[0];
      double dy = transform[1];
      double s = sin(transform[2]);
      double c = cos(transform[2]);

      // perform transform
      for (auto it = trajectory_points_in_one_system->begin();
           it != trajectory_points_in_one_system->end(); ++it) {
        double x = it->x * c + it->y * s + dx;
        double y = it->y * c - it->x * s + dy;
        it->x = x;
        it->y = y;
      }
    }

    // insert the last pointcloud into trajectory_points_in_one_system
    for (auto it = all_points[all_points.size() - 1].get()->begin();
         it != all_points[all_points.size() - 1].get()->end(); ++it) {
      if (!it->is_control_point) {
        trajectory_points_in_one_system->push_back(*it);
      }
    }

    return trajectory_points_in_one_system;
  } else {
    LOG(ERROR) << "three or more files not supported yet, exit.";
    exit(1);
  }
}

#endif
