-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  -- 是否使用[Odometry]消息,为位姿外推器提供位置和姿态
  use_odometry = false,
  -- 要使用的[LaserScan]的个数
  num_laser_scans = 0,
  -- 要使用的[MultiEchoLaserScan]的个数
  num_multi_echo_laser_scans = 1,
  -- 将每个[LaserScan/MultiEchoLaserScan]平均分成十份,封装成RangeData送入上层
  -- TRAJECTORY_BUILDER_2D.scans_per_accumulation 表示将每scans_per_accumulation个
  --       RangeData合成一个scan,做scan-to-map匹配和submap的构造
  -- 上层对RangeData的处理过程详见cartographer中配置文件的注释
  -- 建议该值不要太大
  num_subdivisions_per_laser_scan = 10,
  -- 要使用的[PointCloud2]的个数
  num_point_clouds = 0,
  -- 获取坐标系转换信息的延迟
  lookup_transform_timeout_sec = 0.2,
  -- 发布子图列表[SubmapList]消息的频率(不需要太快)
  submap_publish_period_sec = 0.3,
  -- 发布已匹配点云[PointCloud2]/更新外推器位姿/发布坐标变换消息的频率
  pose_publish_period_sec = 5e-3,
  -- 发布路径和路标[MarkerArray]的频率
  trajectory_publish_period_sec = 30e-3,
  -- rangefinder消息采样比率,为1表示采样所有数据
  rangefinder_sampling_ratio = 1.,
  -- Odometry消息采样比率
  odometry_sampling_ratio = 1.,
  -- Imu消息采样比率
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.scans_per_accumulation = 10

return options
