#include "rclcpp/rclcpp.hpp"

// C++
#include <thread>
#include <fstream>
#include <string>

// ROS
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <processit_msgs/srv/load_task_description.hpp>

using namespace std::chrono_literals;
const rclcpp::Logger LOGGER = rclcpp::get_logger("plugin_task_description_test_node");

class PluginTaskDescriptionTestNode
{
public:
  PluginTaskDescriptionTestNode(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
  }

  void run()
  {
    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools visual_tools("world", "plugin_task_description", node_);
    visual_tools.deleteAllMarkers();

    /*
    geometry_msgs::msg::Pose workpiece_pose;
    std::string filename;
    node_->get_parameter("plugin_task_description_test_node.scene", filename);
    RCLCPP_INFO(LOGGER, "Workpiece filename" + filename);
    // visual_tools.publishMesh(workpiece_pose
    visual_tools.trigger();
    */

    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScene ps(robot_model_loader.getModel());

    // TODO Add hardcoded test case
    // - call service and load exemplary task description
    // - load workpiece to planning scene and publish workpiece tf

    std::string task_description_file = "/home/chl/ws_moveit2/src/UR10e_welding_demo/ipa_demo_support/workpieces/"
                                        "Workpiece_Demo_deviated/Workpiece_Demo_TaskDescription_deviated.xml";

    rclcpp::Client<processit_msgs::srv::LoadTaskDescription>::SharedPtr client =
        node_->create_client<processit_msgs::srv::LoadTaskDescription>("plugin_task_description/load_task_description");

    auto request = std::make_shared<processit_msgs::srv::LoadTaskDescription::Request>();
    request->task_description_file = task_description_file;

    while (!client->wait_for_service(2s))
    {
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }

    processit_msgs::srv::LoadTaskDescription::Response::SharedPtr response;
    auto result = client->async_send_request(request);
    response = result.get();

    RCLCPP_INFO_STREAM(LOGGER, "success" << response->success);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description_test_node");

  PluginTaskDescriptionTestNode plugin_task_description_test_node(node);
  std::thread run_plugin_task_description_test([&plugin_task_description_test_node]() {
    rclcpp::sleep_for(5s);
    plugin_task_description_test_node.run();
  });

  rclcpp::spin(node);
  run_plugin_task_description_test.join();

  return 0;
}