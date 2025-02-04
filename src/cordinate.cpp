#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
//#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
//#include "geometry_msgs/msg/pose.hpp"



class CordinateConverter{
public:
    CordinateConverter(rclcpp::Node::SharedPtr node, const std::string& file_path, const std::string path_type = "ring"){
        node_ = node;
        std::ifstream file(file_path);
        if(!file.is_open()){
            RCLCPP_ERROR(node->get_logger(), "cannot open path file : %s", file_path.c_str());
        } else {
          std::string line_temp;
          while (getline(file, line_temp)) {
            std::stringstream ss(line_temp);
            std::string cell;
            std::vector<double> row;
            int col_counter = 0;
            while (getline(ss, cell, ',') && col_counter < 3) {
              row.push_back(stod(cell));
            }
            global_path_.push_back(row);
          }

          RCLCPP_INFO(node->get_logger(),
                      "successfully load csv path file, %ld points from %s",
                      global_path_.size(), file_path.c_str());
            for(int i=0; i<global_path_.size(); i++){
                global_path_[i].push_back(calcHeading(i));
                global_path_[i].push_back(calcDistance(i));
                
            }
        }
        file.close();


    }
private:
  double calcHeading(int idx) {
    idx = idx % global_path_.size();
    int next_idx = (idx + 1) % global_path_.size();

    double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
    double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

    double dx = next_x - cur_x, dy = next_y - cur_y;
    double theta = atan2(dx, dy);
    return theta;
   }
   double calcDistance(int idx){

    idx = idx % global_path_.size();
    int next_idx = (idx + 1) % global_path_.size();

    double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
    double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];
    double dx = next_x - cur_x, dy = next_y - cur_y;
    
    double distance = sqrt(pow(dx, 2)+pow(dy, 2));
    return distance;
   }
   double calcRechingTime(){

   }

  std::vector<std::vector<double>> global_path_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path::SharedPtr global_path_msg_;

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cordinate_converter");
    CordinateConverter c(node, "/home/rcv/Desktop/cordinate/src/cordinate/path/4f_floor.csv");
    //clcpp::spin();
}