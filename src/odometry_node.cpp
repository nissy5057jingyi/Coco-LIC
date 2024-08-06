/*
 * Coco-LIC: Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once
#include <ros/package.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <odom/odometry_manager.h>

// lrq
#include "opencv2/opencv.hpp"

using namespace cocolic;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "cocolic");
  ros::NodeHandle nh("~");

  std::string config_path;
  nh.param<std::string>("config_path", config_path, "ct_odometry.yaml");
  ROS_INFO("Odometry load %s.", config_path.c_str());

  YAML::Node config_node = YAML::LoadFile(config_path);

  std::string log_path = config_node["log_path"].as<std::string>();
  FLAGS_log_dir = log_path;
  FLAGS_colorlogtostderr = true;
  std::cout << "\n🥥 Start Coco-LIC Odometry 🥥";

  OdometryManager odom_manager(config_node, nh);
  pcl::PointCloud<pcl::PointXYZI> map_cloud;
  odom_manager.RunBag(map_cloud);

  std::string file_path = "/home/dell/test/tmp.pcd";

  pcl::io::savePCDFileASCII(file_path, map_cloud);
  std::cout << "Saved " << map_cloud.size() << " data points to visual_sub_map_debug.pcd." << std::endl;

  double t_traj_max = odom_manager.SaveOdometry();


  // lrq
  //char ch = odom_manager.cv_keyboard_callback();
  cv::Mat Control_panel = odom_manager.generate_control_panel_img();
  std::cout << "\n Control_panel \n";
  cv::imshow("Control panel", Control_panel);
  cv::waitKey(0);

  // odom_manager.generate_control_panel_img();


  std::cout << "\n✨ All Done.\n\n";

  return 0;
}
