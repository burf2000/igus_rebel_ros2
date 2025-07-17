#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <memory>

using json = nlohmann::json;

class ArmApiNode {
public:
  ArmApiNode(rclcpp::Node::SharedPtr nh)
  : nh_(nh),
    move_group_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(nh_, "igus_rebel_arm"))
  {
    server_.set_logger([](const httplib::Request& req, const httplib::Response& res) {
      printf("[HTTP] %s %s -> %d\n", req.method.c_str(), req.path.c_str(), res.status);
    });

    server_.set_error_handler([](const httplib::Request&, httplib::Response& res) {
      res.set_content("Not found", "text/plain");
      res.status = 404;
    });

    server_.Post("/move", [this](const httplib::Request& req, httplib::Response& res) {
      RCLCPP_INFO(nh_->get_logger(), "[/move POST] body: %s", req.body.c_str());
      try {
        auto j = json::parse(req.body);
        geometry_msgs::msg::Pose target;
        target.position.x = j.at("posX");
        target.position.y = j.at("posY");
        target.position.z = j.at("posZ");
        target.orientation.x = j.at("rotX");
        target.orientation.y = j.at("rotY");
        target.orientation.z = j.at("rotZ");
        target.orientation.w = j.at("rotW");

        RCLCPP_INFO(nh_->get_logger(),
          "Parsed pose: pos(%.3f,%.3f,%.3f) rot(%.3f,%.3f,%.3f,%.3f)",
          target.position.x, target.position.y, target.position.z,
          target.orientation.x, target.orientation.y,
          target.orientation.z, target.orientation.w);

        move_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_res = move_group_->plan(plan);
        auto exec_res = move_group_->execute(plan.trajectory);
        
        if (plan_res == moveit::core::MoveItErrorCode::SUCCESS &&
            exec_res == moveit::core::MoveItErrorCode::SUCCESS)
        {
          res.set_content("Motion executed", "text/plain");
          RCLCPP_INFO(nh_->get_logger(), "Motion executed");
        } else {
          res.set_content("Planning or execution failed", "text/plain");
          RCLCPP_WARN(nh_->get_logger(), "Planning/execution failed");
        }
      } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(std::string("JSON error: ") + e.what(), "text/plain");
        RCLCPP_ERROR(nh_->get_logger(), "JSON parse error: %s", e.what());
      }
    });

    server_.Get("/move", [&](const httplib::Request& req, httplib::Response& res) {
      // Quick test endpoint
      res.set_content(
        R"({"posX":0.3,"posY":0.2,"posZ":0.4,"rotX":0,"rotY":0,"rotZ":0,"rotW":1})",
        "application/json");
      RCLCPP_INFO(nh_->get_logger(), "[/move GET] responded with template JSON");
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
