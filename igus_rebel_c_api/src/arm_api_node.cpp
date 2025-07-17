#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <httplib.h>
#include <memory>
#include <thread>

class ArmApiNode : public rclcpp::Node,
                   public std::enable_shared_from_this<ArmApiNode>
{
  using MoveGroup = moveit::planning_interface::MoveGroupInterface;
public:
  ArmApiNode() : Node("arm_api_node") {}

  void init()
  {
    // explicit ARMAPI class shared_from_this
    auto self = std::enable_shared_from_this<ArmApiNode>::shared_from_this();
    move_group = std::make_shared<MoveGroup>(self, "igus_rebel_arm");

    server.Post("/move", [self, this](const httplib::Request &req, httplib::Response &res) {
      double x = std::stod(req.get_param_value("x"));
      double y = std::stod(req.get_param_value("y"));
      double z = std::stod(req.get_param_value("z"));
      geometry_msgs::msg::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = z;

      target.orientation.w = 1.0;
      move_group->setPoseTarget(target);

      MoveGroup::Plan plan;
      auto success = move_group->plan(plan);
      if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        auto exec_res = move_group->execute(plan.trajectory);
        res.set_content(
          exec_res == moveit::core::MoveItErrorCode::SUCCESS ?
            "Motion executed" : "Execution failed",
          "text/plain"
        );
      } else {
        res.set_content("Planning failed", "text/plain");
      }
    });

    // capture the same self shared_ptr
    std::thread([self]() {
      RCLCPP_INFO(self->get_logger(), "HTTP server running on port 8080");
      self->server.listen("0.0.0.0", 8080);
    }).detach();
  }

private:
  std::shared_ptr<MoveGroup> move_group;
  httplib::Server server;
};

inline std::shared_ptr<ArmApiNode> make_arm_api_node()
{
  auto node = std::make_shared<ArmApiNode>();
  node->init();
  return node;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = make_arm_api_node();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
