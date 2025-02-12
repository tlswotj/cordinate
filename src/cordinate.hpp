#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
//#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
//#include "pgm_to_occupancy_grid_node.cpp"
//#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "geometry_msgs/msg/pose.hpp"

class CordinateConverter {
public:
  CordinateConverter(rclcpp::Node::SharedPtr node, std::string config_file_path,
                     const std::string path_type);

private:
  void readPath(std::string path_file_path);

  double calcHeading(int idx);

  double calcDistance(int idx);

  double calcRechingTime(int idx);

  std::vector<std::vector<double>> global_path_;
  std::vector<double> global_path_heading_;
  std::vector<double> global_path_distance_;
  std::vector<double> global_path_reaching_time_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path::SharedPtr global_path_msg_;
};
