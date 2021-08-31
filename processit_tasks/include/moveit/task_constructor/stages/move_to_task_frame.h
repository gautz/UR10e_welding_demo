#pragma once
// #ifndef MOVE_TO_TASK_FRAME_H
// #define MOVE_TO_TASK_FRAME_H
#include "rclcpp/rclcpp.hpp"

#include <moveit_msgs/msg/Constraints.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <processit_msgs/msg/weld_seam.h>
#include <processit_msgs/srv/load_task_description.h>
// #include <processit_cax/plugin_task_description.h>
// #include <processit_cax/TaskDescription.h>

using namespace std::chrono_literals;
const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_task_frame");

// namespace moveit
// {
// 	namespace core
// 	{
// 		class RobotState;
// 	}
// } // namespace moveit
namespace moveit
{
namespace task_constructor
{
namespace stages
{
// namespace processit_tasks
// {
// using namespace hybrid_planning_demo;

/** Perform a Motion to a task frame given a Feature Frame and FeatureToTask Transformation */
class MoveToTaskFrame : public MoveTo
{
public:
  MoveToTaskFrame(const std::string& name = "move to task frame",
                  const solvers::PlannerInterfacePtr& planner = solvers::PlannerInterfacePtr(),
                  const rclcpp::Node::SharedPtr& node)
    : MoveTo(name, planner)
  {
    node_ = node;
  }

  void startState(int feature_id, bool reverse_direction);

  void getFeatures();
  void setTask(int feature_id, bool reverse_direction);

  /// Set line constraints for OMPL Constrained Planning
  moveit_msgs::msg::Constraints setLinConstraints();

  /// get constraints for a given feature frame
  moveit_msgs::msg::Constraints setCircConstraints(const std::string& feature_frame);
  processit_msgs::srv::LoadTaskDescription::Response current_features_;
  processit_msgs::msg::WeldSeam getTaskFrames(int feature_id);

private:
  rclcpp::Node::SharedPtr node_;
  const std::string world_frame_ = "world";                       // Default world frame
  const std::string workpiece_frame_ = "workpiece";               // Default workpiece frame
  const std::string welding_tcp_frame_ = "tcp_welding_gun_link";  // Default TCP frame
  const std::string path_constraints_name_ =
      "linear_system_constraints";                            // Default path constraints for OMPL constrained planner.
                                                              // Alternative: "use_equality_constraints"
  const std::string pilz_circ_third_point_type_ = "interim";  // Default third point type for Pilz CIRC motion
  geometry_msgs::msg::PoseStamped start_frame_;
  geometry_msgs::msg::PoseStamped end_frame_;
  geometry_msgs::msg::PoseStamped interim_frame_;
  geometry_msgs::msg::Transform identity_transform_;

protected:
};
// } // namespace processit_tasks
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
