#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
//#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "cordinate.hpp"
#include "nav_msgs/msg/path.hpp"
//#include "pgm_to_occupancy_grid_node.cpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

CordinateConverter::CordinateConverter(rclcpp::Node::SharedPtr node,
                                       bool node_mode = false,
                                       std::string config_file_path = "",
                                       const std::string path_type = "ring") {
  node_ = node;
  node_mode_ = node_mode;

  if (node_mode_) {
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
      path_msg_generator();

      file.close();
      map_node_ =
          new OccupancyGridNode(node_, pgm_file_path, map_yaml_file_path);
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local();
      qos.reliable();

      publisher_ =
          node_->create_publisher<nav_msgs::msg::Path>("global_path", qos);
      timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        path_publisher();
        timer_->cancel();
      });
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "waiting for global path topic");
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();
    subscriber_ = node_->create_subscription<nav_msgs::msg::Path>(
        "global_path", qos, [this](const nav_msgs::msg::Path::SharedPtr msg) {
          pathCallback(*msg);
        });
  }
}

CordinateConverter::~CordinateConverter() {
  if (node_mode_) {
    delete map_node_;
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
    global_path_heading_.push_back(calcPathToPathHeading(i));
    global_path_distance_.push_back(calcPathToPathDistance(i));
    global_path_reaching_time_.push_back(calcPathToPathRechingTime(i));
  }
}

void CordinateConverter::readPath(nav_msgs::msg::Path path_topic) {
  for (int i = 0; i < path_topic.poses.size(); i++) {
    std::vector<double> temp;
    temp.push_back(path_topic.poses[i].pose.position.x);
    temp.push_back(path_topic.poses[i].pose.position.y);
    temp.push_back(extract_speed(path_topic.poses[i].header.stamp));
    global_path_.push_back(temp);
  }
  for (int i = 0; i < global_path_.size(); i++) {
    global_path_heading_.push_back(calcPathToPathHeading(i));
    global_path_distance_.push_back(calcPathToPathDistance(i));
    global_path_reaching_time_.push_back(calcPathToPathRechingTime(i));
  }
}

double CordinateConverter::extract_speed(builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) / 1e9;
}

double CordinateConverter::calcPathToPathHeading(int idx) {
  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  double dx = next_x - cur_x, dy = next_y - cur_y;
  double theta = atan2(dy, dx);
  return theta;
}

double CordinateConverter::calcPathToPathDistance(int idx) {

  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  return calcDistance(cur_x, cur_y, next_x, next_y);
}

double CordinateConverter::calcPathToPathRechingTime(int idx) {
  idx = idx % global_path_.size();
  int prev_idx = ((idx - 1) + global_path_.size()) % global_path_.size();
  double speed = global_path_[idx][2], prev_speed = global_path_[prev_idx][2];
  double avg_speed = (speed + prev_speed) / 2;
  return calcPathToPathDistance(prev_idx) / avg_speed;
}

int CordinateConverter::getClosestsIndex(double x, double y) {
  double closest_dist = INFINITY;
  int idx;
  for (int i = 0; i < global_path_.size(); i++) {
    double dist = calcDistance(global_path_[i][0], global_path_[i][1], x, y);
    if (closest_dist > dist) {
      closest_dist = dist;
      idx = i;
    }
  }
  return idx;
}

std::vector<double> CordinateConverter::globalToFrenel(double x, double y) {
  int closest_idx = getClosestsIndex(x, y);

  double dist = calcDistance(x, y, global_path_[closest_idx][0],
                             global_path_[closest_idx][1]);
  double s = calcPathDistance(0, closest_idx);
  std::vector<double> output;
  output.push_back(s);
  output.push_back(dist);
  return output;
}

double CordinateConverter::calcPathDistance(int idx_start, int idx_end) {
  double distance_counter = 0;
  for (int idx = idx_start; idx < idx_end; idx++) {
    distance_counter += calcPathToPathDistance(idx);
  }
  return distance_counter;
}

double CordinateConverter::calcDistance(double x, double y, double x1,
                                        double y1) {
  return sqrt(pow(x - x1, 2) + pow(y - y1, 2));
}

void CordinateConverter::path_publisher() {
  publisher_->publish(*global_path_msg_);
  RCLCPP_INFO(node_->get_logger(), "Published Path with %zu points",
              global_path_msg_->poses.size());
}

void CordinateConverter::path_msg_generator() {
  global_path_msg_ = std::make_shared<nav_msgs::msg::Path>();
  global_path_msg_->header.stamp = node_->get_clock()->now();
  global_path_msg_->header.frame_id = "map";

  for (int i = 0; i < global_path_.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp.sec = static_cast<int32_t>(global_path_[i][2]);
    pose.header.stamp.nanosec = static_cast<int32_t>(
        (global_path_[i][2] - pose.header.stamp.sec) * 1e9);
    pose.header.frame_id = "map";
    pose.pose.position.x = global_path_[i][0];
    pose.pose.position.y = global_path_[i][1];
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = global_path_heading_[i];
    pose.pose.orientation.w = 1.0; // 단순한 예제이므로 회전 없음
    global_path_msg_->poses.push_back(pose);
  }
  global_path_msg_;
}

void CordinateConverter::pathCallback(nav_msgs::msg::Path msg) {
  global_path_msg_ = std::make_shared<nav_msgs::msg::Path>(msg);
}

std::vector<std::vector<double>> CordinateConverter::getGlobalPath() {
  return global_path_;
}

std::pair<double, double> CordinateConverter::calcProj(int idx, int next_idx,
                                                       double x, double y) {
  double x1 = global_path_[idx][0], y1 = global_path_[idx][1];
  double x2 = global_path_[next_idx][0], y2 = global_path_[next_idx][1];
  double x3 = x, y3 = y;
  double ABx = x2 - x1;
  double ABy = y2 - y1;
  // 벡터 AC = (x3 - x1, y3 - y1)
  double ACx = x3 - x1;
  double ACy = y3 - y1;

  // 내적(AB·AC) / 내적(AB·AB)
  double dotABAC = ABx * ACx + ABy * ACy;

  double dotABAB = ABx * ABx + ABy * ABy;

  double t = dotABAC / dotABAB;

  // 발 D의 좌표 (Dx, Dy)
  double Dx = x1 + t * ABx;
  double Dy = y1 + t * ABy;

  // CD 거리
  double distCD = hypot(x3 - Dx, y3 - Dy);

  // A에서 발 D까지 거리
  double distAD = hypot(Dx - x1, Dy - y1);

  return std::make_pair(distCD, distAD);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cordinate_converter");
  CordinateConverter c(node, true);
  std::vector<double> a = c.globalToFrenel(3.0, 2.0);
  RCLCPP_INFO(node->get_logger(), "frenet frame : s %3.f, d %3.f", a[0], a[1]);
  rclcpp::spin(node);
}