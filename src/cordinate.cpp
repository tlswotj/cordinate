#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
//#include "std_msgs/msg/header.hpp"
#include "cordinate.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pgm_to_occupancy_grid_node.cpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "geometry_msgs/msg/pose.hpp"

CordinateConverter::CordinateConverter(rclcpp::Node::SharedPtr node,
                                       std::string config_file_path = "",
                                       const std::string path_type = "ring") {
  node_ = node;
  if (config_file_path == "") {
    config_file_path =
        ament_index_cpp::get_package_share_directory("cordinate") +
        "/config/config.yaml";
  }
  std::string path_file_path, pgm_file_path, map_yaml_file_path;

  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "cannot open path file : %s",
                 config_file_path.c_str());
  } else {
    std::string yaml_line;
    while (std::getline(file, yaml_line)) {
      std::istringstream ss(yaml_line);
      std::string key;
      ss >> key;
      if (key == "path:") {
        ss >> path_file_path;
      } else if (key == "map_yaml:") {
        ss >> map_yaml_file_path;
      } else if (key == "map_pgm:") {
        ss >> pgm_file_path;
      }
    }

    readPath(path_file_path);

    file.close();
  }
}

void CordinateConverter::readPath(std::string path_file_path) {
  std::ifstream path_file(path_file_path);
  std::string line_temp;
  while (getline(path_file, line_temp)) {
    std::stringstream ss(line_temp);
    std::string cell;
    std::vector<double> row;
    int col_counter = 0;
    while (getline(ss, cell, ',') && col_counter < 3) {
      row.push_back(stod(cell));
    }
    global_path_.push_back(row);
  }

  RCLCPP_INFO(node_->get_logger(),
              "successfully load csv path file, %ld points from %s",
              global_path_.size(), path_file_path.c_str());
  for (int i = 0; i < global_path_.size(); i++) {
    global_path_heading_.push_back(calcHeading(i));
    global_path_distance_.push_back(calcDistance(i));
    global_path_reaching_time_.push_back(calcRechingTime(i));
  }
}

double CordinateConverter::calcHeading(int idx) {
  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  double dx = next_x - cur_x, dy = next_y - cur_y;
  double theta = atan2(dx, dy);
  return theta;
}

double CordinateConverter::calcDistance(int idx) {

  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];
  double dx = next_x - cur_x, dy = next_y - cur_y;

  double distance = sqrt(pow(dx, 2) + pow(dy, 2));
  return distance;
}

double CordinateConverter::calcRechingTime(int idx) {
  idx = idx % global_path_.size();
  int prev_idx = ((idx - 1) + global_path_.size()) % global_path_.size();
  double speed = global_path_[idx][2], prev_speed = global_path_[prev_idx][2];
  double avg_speed = (speed + prev_speed) / 2;
  return calcDistance(prev_idx) / avg_speed;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cordinate_converter");
  CordinateConverter c(node);
  // clcpp::spin();
}