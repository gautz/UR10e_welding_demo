#include <numeric>
// ROS
// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>

#include <moveit/task_constructor/stages/move_to_task_frame.h>
// #include <processit_cax/plugin_task_description.h>
#include <tf2_eigen/tf2_eigen.h>

namespace moveit
{
namespace task_constructor
{
namespace stages
{
// namespace processit_tasks
// {

// processit_msgs::WeldSeam[]
void MoveToTaskFrame::getFeatures()
{
  std::string workpiece_path = "";
  node_->get_parameter("workpiece_path", workpiece_path);

  // TODO currently only hardcoded pose
  Eigen::Isometry3d workpiece_pose = Eigen::Isometry3d::Identity();
  workpiece_pose.translation().x() = 0.1;
  workpiece_pose.translation().y() = -0.2;
  workpiece_pose.translation().z() = 0.71;

  // Service call to load a task description
  rclcpp::Client<processit_msgs::srv::LoadTaskDescription>::SharedPtr client =
      node_->create_client<processit_msgs::srv::LoadTaskDescription>("plugin_task_description/load_task_description");
  auto request = std::make_shared<processit_msgs::srv::LoadTaskDescription::Request>();

  // Set task description filename
  std::string task_file = workpiece_path + ".xml";
  RCLCPP_INFO(LOGGER, "Loading task " + task_file);
  request->task_description_file = task_file;

  // Set workpiece pose (task description is relative to workpiece frame)
  geometry_msgs::msg::PoseStamped workpiece_pose_stamped;
  workpiece_pose_stamped.header.frame_id = "world";
  workpiece_pose_stamped.pose = tf2::toMsg(workpiece_pose);
  request->workpiece_pose = workpiece_pose_stamped;

  while (!client->wait_for_service(1s))
  {
    RCLCPP_INFO(LOGGER, "service not available, waiting again...");
  }

  processit_msgs::srv::LoadTaskDescription::Response::SharedPtr response;
  auto result = client->async_send_request(request);
  response = result.get();

  RCLCPP_INFO_STREAM(LOGGER, "success" << response->success);
  current_features_ = response->weld_seams;
  // return response->weld_seams
}

processit_msgs::msg::WeldSeam MoveToTaskFrame::getTaskFrames(int feature_id)
{
  return current_features_[feature_id];
}

/// Get pose of task frame from task description service
// geometry_msgs::PoseStamped MoveToTaskFrame::getTaskFrame(int feature_id, int pose_id)
// {
// 	geometry_msgs::PoseStamped task_frame;
// 	task_frame.pose = current_features_[feature_id].poses[pose_id];
// 	RCLCPP_DEBUG_STREAM(LOGGER, "Weld seam position " + std::to_string(task_frame.pose.position.x) + "  " +
//                     std::to_string(task_frame.pose.position.y) + +"  " + std::to_string(task_frame.pose.position.z));
// 	return task_frame;
// }

void MoveToTaskFrame::startState(int feature_id, bool reverse_direction)
{
  getFeatures();
  processit_msgs::msg::WeldSeam task_frames = getTaskFrames(feature_id);

  if (!reverse_direction)
  {
    start_frame_.pose = task_frames.poses[0];
    setProperty("goal", start_frame_);
  }
  else
  {
    end_frame_.pose = task_frames.poses[1];
    setProperty("goal", end_frame_);
  }
}

void MoveToTaskFrame::setTask(int feature_id, bool reverse_direction)
{
  getFeatures();
  processit_msgs::msg::WeldSeam task_frames = getTaskFrames(feature_id);

  if (!reverse_direction)
  {
    start_frame_.pose = task_frames.poses[0];
    end_frame_.pose = task_frames.poses[1];
    setProperty("goal", end_frame_);
  }
  else
  {
    start_frame_.pose = task_frames.poses[1];
    end_frame_.pose = task_frames.poses[0];
    setProperty("goal", start_frame_);
  }
  // Linear Feature
  if (task_frames.poses.size() == 2)
  {
    if (planner_plugin == "ompl_interface/OMPLPlanner")
    {
      setLinConstraints();
    }
  }
  // Circular Feature
  else if (task_frames.poses.size() == 3)
    // Spline Feature
    else if (task_frames.poses.size() > 3)
}

/// Set line constraints for OMPL Constrained Planning
moveit_msgs::Constraints MoveToTaskFrame::setLinConstraints()
{
  geometry_msgs::msg::PoseStamped start_frame = start_frame_;
  geometry_msgs::msg::PoseStamped goal_frame = end_frame_;
  moveit_msgs::Constraints path_constraints;
  moveit_msgs::PositionConstraint pos_constraint;

  std::vector<double> start_goal_vector;
  start_goal_vector = { goal_frame.pose.position.x - start_frame.pose.position.x,
                        goal_frame.pose.position.y - start_frame.pose.position.y,
                        goal_frame.pose.position.z - start_frame.pose.position.z };
  float distance = sqrtf(std::inner_product(std::begin(start_goal_vector), std::end(start_goal_vector),
                                            std::begin(start_goal_vector), 0.0));
  shape_msgs::SolidPrimitive constraints_box;
  constraints_box.type = shape_msgs::SolidPrimitive::BOX;
  constraints_box.dimensions = { distance * 1.1, distance * 1.1, distance * 1.1 };
  pos_constraint.constraint_region.primitives.push_back(constraints_box);

  geometry_msgs::msg::Pose constraints_middle_frame;
  constraints_middle_frame.position.x = start_frame.pose.position.x + start_goal_vector[0] / 2;
  constraints_middle_frame.position.y = start_frame.pose.position.y + start_goal_vector[1] / 2;
  constraints_middle_frame.position.z = start_frame.pose.position.z + start_goal_vector[2] / 2;
  pos_constraint.constraint_region.primitive_poses.push_back(constraints_middle_frame);

  // Hack: currently no solution is found if the equation system of the line is defined by start and goal point.
  // The start and goal poses are probably considered as singularities
  goal_frame.pose.position.x += 0.1 * start_goal_vector[0];
  goal_frame.pose.position.y += 0.1 * start_goal_vector[1];
  goal_frame.pose.position.z += 0.1 * start_goal_vector[2];
  start_frame.pose.position.x -= 0.1 * start_goal_vector[0];
  start_frame.pose.position.y -= 0.1 * start_goal_vector[1];
  start_frame.pose.position.z -= 0.1 * start_goal_vector[2];
  // Hack: currently no solution is found if the equation system of the line is defined by start and goal point.

  pos_constraint.header.frame_id = world_frame_;
  pos_constraint.link_name = welding_tcp_frame_;
  path_constraints.name = path_constraints_name_;
  pos_constraint.constraint_region.primitive_poses.push_back(start_frame.pose);
  pos_constraint.constraint_region.primitive_poses.push_back(goal_frame.pose);
  path_constraints.position_constraints.push_back(pos_constraint);
  // setProperty("path_constraints", std::move(path_constraints));
  return path_constraints;
  RCLCPP_DEBUG(LOGGER, "Path constraints set for LIN");
}

/// Set CIRC constraints for OMPL Constrained Planning or PILZ industrial motion planner
moveit_msgs::Constraints MoveToTaskFrame::setCircConstraints(const std::string& planner_plugin)
{
  moveit_msgs::Constraints path_constraints;
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = world_frame_;
  pos_constraint.link_name = welding_tcp_frame_;
  if (planner_plugin == "pilz_industrial_motion_planner::CommandPlanner")
  {
    // Set constraints for Pilz circular motion
    pos_constraint.constraint_region.primitive_poses.push_back(interim_frame_.pose);
    path_constraints.name = pilz_circ_third_point_type_;
  }
  else if (planner_plugin == "ompl_interface/OMPLPlanner")
  {
    // Set constraints for OMPL constrained circular motion
    path_constraints.name = path_constraints_name_;

    std::vector<double> start_goal_vector;
    start_goal_vector = { end_frame_.pose.position.x - start_frame_.pose.position.x,
                          end_frame_.pose.position.y - start_frame_.pose.position.y,
                          end_frame_.pose.position.z - start_frame_.pose.position.z };
    float distance = sqrtf(std::inner_product(std::begin(start_goal_vector), std::end(start_goal_vector),
                                              std::begin(start_goal_vector), 0.0));
    shape_msgs::SolidPrimitive constraints_box;
    constraints_box.type = shape_msgs::SolidPrimitive::BOX;
    constraints_box.dimensions = { distance * 1.1, distance * 1.1, distance * 1.1 };
    pos_constraint.constraint_region.primitives.push_back(constraints_box);

    geometry_msgs::msg::Pose constraints_middle_frame;
    constraints_middle_frame.position.x = start_frame_.pose.position.x + start_goal_vector[0] / 2;
    constraints_middle_frame.position.y = start_frame_.pose.position.y + start_goal_vector[1] / 2;
    constraints_middle_frame.position.z = start_frame_.pose.position.z + start_goal_vector[2] / 2;
    pos_constraint.constraint_region.primitive_poses.push_back(constraints_middle_frame);

    pos_constraint.constraint_region.primitive_poses.push_back(start_frame_.pose);
    pos_constraint.constraint_region.primitive_poses.push_back(end_frame_.pose);
    pos_constraint.constraint_region.primitive_poses.push_back(interim_frame_.pose);
  }
  path_constraints.position_constraints.push_back(pos_constraint);
  RCLCPP_DEBUG(LOGGER, "Path constraints set for CIRC");
  return path_constraints;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
