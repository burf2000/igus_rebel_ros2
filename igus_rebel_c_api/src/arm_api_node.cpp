#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <httplib.h>
#include <memory>
#include <thread>

class ArmApiNode {
public:
  ArmApiNode(rclcpp::Node::SharedPtr nh)
  : nh_(nh),
    move_group_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(nh_, "igus_rebel_arm"))
  {
    server_.set_logger([](const httplib::Request& req, const httplib::Response& res) {
      printf("[HTTP] %s %s -> %d\n", req.method.c_str(), req.path.c_str(), res.status);
    });

    server_.set_error_handler([](const httplib::Request& /*req*/, httplib::Response& res) {
      res.set_content("Not found", "text/plain");
      res.status = 404;
    });

    server_.Post("/move", [&](const httplib::Request& req, httplib::Response& res) {
      RCLCPP_INFO(nh_->get_logger(), "[/move] called: %s", req.path.c_str());
      double x = std::stod(req.get_param_value("x"));
      double y = std::stod(req.get_param_value("y"));
      double z = std::stod(req.get_param_value("z"));
      RCLCPP_INFO(nh_->get_logger(), "Coords: x=%.3f y=%.3f z=%.3f", x, y, z);

      geometry_msgs::msg::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = z;
      target.orientation.w = 1.0;
      move_group_->setPoseTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          move_group_->execute(plan.trajectory) == moveit::core::MoveItErrorCode::SUCCESS) {
        res.set_content("Motion executed", "text/plain");
        RCLCPP_INFO(nh_->get_logger(), "Motion executed");
      } else {
        res.set_content("Planning/execution failed", "text/plain");
        RCLCPP_WARN(nh_->get_logger(), "Planning/execution failed");
      }
    });

    std::thread([this]() {
      RCLCPP_INFO(nh_->get_logger(), "Starting HTTP server on port 8080");
      server_.listen("0.0.0.0", 8080);
    }).detach();
  }

private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  httplib::Server server_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("arm_api_node");
  ArmApiNode api(nh);
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
