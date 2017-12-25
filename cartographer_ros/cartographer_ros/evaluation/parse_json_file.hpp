#ifndef PARSE_JSON_HPP
#define PARSE_JSON_HPP

#include "boost/foreach.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "boost/property_tree/ptree.hpp"
#include "cartographer/common/time.h"
#include "glog/logging.h"
#include <memory>
#include <sstream>

struct Point {
  std::string name;
  cartographer::common::Time timestamp;
  bool is_control_point;
  double x;
  double y;
  double z;
};

typedef std::vector<Point> Points;

typedef std::shared_ptr<Points> PointsPtr;

std::ostream &operator<<(std::ostream &out, Point &p) {
  out << p.name << " " << p.timestamp << " " << p.is_control_point << " " << p.x
      << " " << p.y << " " << p.z;
  return out;
}

// 解析含有时间戳的点坐标数据文件，返回点集
PointsPtr ParseJsonFile(const std::string &filename) {
  namespace pt = boost::property_tree;

  PointsPtr points_ptr(new Points);

  pt::ptree tree;
  pt::read_json(filename, tree);
  auto arr = tree.get_child("points");

  std::stringstream ss;
  BOOST_FOREACH (pt::ptree::value_type &v, arr) {
    pt::ptree p = v.second;
    auto name = p.get<std::string>("name");
    auto timestamp = p.get<std::string>("timestamp");
    auto is_control_point = p.get<bool>("is_control_point");
    auto x = p.get<double>("x");
    auto y = p.get<double>("y");
    auto z = p.get<double>("z");

    // parse timestamp to cartographer::common::Time
    unsigned long year, month, day, hour, minute, second, nanosecond;
    ss.clear();
    ss << timestamp;
    ss >> year >> month >> day >> hour >> minute >> second >> nanosecond;

    // cartographer::common::Time 以 100ns 为一个最小单位
    cartographer::common::Time time = cartographer::common::FromUniversal(
        (hour * 3600 + minute * 60 + second) * 10000000 + nanosecond / 100);

    points_ptr->push_back({name, time, is_control_point, x, y, z});
  }

  return points_ptr;
}

bool WriteJsonFile(const std::string &filename, const PointsPtr &points_ptr,
                   const std::vector<std::vector<double>> &trajecotry_nodes,
                   const std::vector<double> &transform) {
  namespace pt = boost::property_tree;

  pt::ptree root, trajecotry_nodes_node, points_node, transform_node;

  for (auto it = points_ptr->begin(); it != points_ptr->end(); ++it) {
    pt::ptree point_node;
    point_node.put("name", it->name);
    point_node.put("timestamp",
                   cartographer::common::ToUniversal(it->timestamp));
    point_node.put("is_control_point", it->is_control_point);
    point_node.put("x", it->x);
    point_node.put("y", it->y);
    point_node.put("z", it->z);
    points_node.push_back(make_pair("", point_node));
  }

  root.put_child("points", points_node);

  for (auto it = trajecotry_nodes.begin(); it != trajecotry_nodes.end(); ++it) {
  }

  root.put_child("trajecotry_nodes", trajecotry_nodes_node);

  {
    transform_node.put("x", transform[0]);
    transform_node.put("y", transform[1]);
    transform_node.put("theta", transform[2]);
    transform_node.put("delta_time", int64(transform[2]));
  }

  root.put_child("transform", transform_node);

  pt::write_json(filename, root);

  return true;
}

#endif
