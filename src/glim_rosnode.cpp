#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  rclcpp::spin(glim);
  rclcpp::shutdown();

  // std::string dump_path = "/tmp/dump";

  // glim->wait();
  // glim->save(dump_path);
  // glim.reset();

  return 0;
}