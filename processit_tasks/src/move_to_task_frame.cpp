#include <numeric>
// ROS
// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>

#include <moveit/task_constructor/stages/move_to_task_frame.h>
#include <processit_cax/plugin_task_description.h>

namespace moveit
{
namespace task_constructor
{
namespace stages
{
// namespace processit_tasks
// {

// bool MoveToTaskFrame::getCurrentWorkpieceFrame(const std::string& feature_frame)
// {
//   // std::string workpiece_frame;
//   bool parent_exists;
//   try
//   {
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     // Get the workpiece on which the feature is located
//     // parent_exists = tfBuffer._getParent(feature_frame, ros::Time::now(), workpiece_frame_);
//   }
//   catch (tf2::TransformException& ex)
//   {
//     // If no workpiece frame is available, do not change frames.
//     RCLCPP_WARN_STREAM(LOGGER, "Exception while getting parent of:" << feature_frame);
//   }
//   return parent_exists;
// }

// geometry_msgs::TransformStamped MoveToTaskFrame::getFeatureToTaskTransform(const std::string& feature_to_task_frame)
// {
//   geometry_msgs::TransformStamped feature_to_task_transform_stamped;
//   try
//   {
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     // Load the  feature_to_task_frame transform from the TF frame, child of the workpiece frame
//     feature_to_task_transform_stamped = tfBuffer.lookupTransform(
//         workpiece_frame_, feature_to_task_frame, ros::Time::now(), ros::Duration(3.0));  // ros::Time(0),
//   }
//   catch (tf2::TransformException& ex)
//   {
//     // If no workpiece frame is available, do not change frames.
//     RCLCPP_WARN_STREAM(LOGGER, "Exception while looking up feature_to_task_transform:" << feature_to_task_frame);
//   }
//   return feature_to_task_transform_stamped;
// }

// processit_msgs::WeldSeam[]
void MoveToTaskFrame::getFeatures()
{
  processit_msgs::srv::LoadTaskDescription::Response::SharedPtr response;
  auto result = client->async_send_request(request);
  response = result.get();

  RCLCPP_INFO_STREAM(LOGGER, "success" << response->success);
  current_features_ = response->weld_seams;
  // return response->weld_seams
}

geometry_msgs::Pose[] MoveToTaskFrame::getTaskFrames(int feature_id)
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

// /// Get pose of task frame with specified TF Feature frame name and TF FeatureToTask frame name from technology model
// /// and additional offsets for e.g. approach/retrieve
// geometry_msgs::PoseStamped MoveToTaskFrame::getTaskFrame(const std::string& feature_frame,
//                                                          const std::string& feature_to_task_frame,
//                                                          const geometry_msgs::Transform offset_transform)
// {
//   geometry_msgs::TransformStamped task_transform_stamped;
//   task_transform_stamped = getFeatureToTaskTransform(feature_to_task_frame);
//   geometry_msgs::PoseStamped task_frame =
//       MoveToTaskFrame::getTaskFrame(feature_frame, task_transform_stamped.transform, offset_transform);
//   return task_frame;
// }

// /// Get pose of task frame with specified TF Feature frame name and FeatureToTask transformation and additional offsets
// /// for e.g. approach/retrieve
// geometry_msgs::PoseStamped MoveToTaskFrame::getTaskFrame(const std::string& feature_frame,
//                                                          const geometry_msgs::Transform feature_to_task_transform,
//                                                          const geometry_msgs::Transform offset_transform)
// {
//   geometry_msgs::PoseStamped task_frame;
//   try
//   {
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     task_frame.header.frame_id = feature_frame;
//     // Standard welding gun orientation and stickout from the technology model
//     task_frame.pose.position.x = feature_to_task_transform.translation.x + offset_transform.translation.x;
//     task_frame.pose.position.y = feature_to_task_transform.translation.y + offset_transform.translation.y;
//     task_frame.pose.position.z = feature_to_task_transform.translation.z + offset_transform.translation.z;
//     task_frame.pose.orientation =
//         feature_to_task_transform.rotation;  // TODO implement composition of rotations + offset_transform.rotation
//     task_frame = tfBuffer.transform(task_frame, world_frame_, ros::Duration(3.0));
//   }
//   catch (tf2::TransformException& ex)
//   {
//     // If no workpiece frame is available, do not change frames.
//     RCLCPP_WARN_STREAM(LOGGER, "Exception while getting task frame:" << feature_frame);
//   }
//   return task_frame;
// }

void MoveToTaskFrame::startState(int feature_id, bool reverse_direction)
{
  getFeatures();
  geometry_msgs::Pose[] task_frames = getTaskFrames(feature_id);

  if (!reverse_direction)
  {
    start_frame_.pose = task_frames[0];
    setProperty("goal", start_frame_);
  }
  else
  {
    end_frame_.pose = task_frames[1];
    setProperty("goal", end_frame_);
  }
}

void MoveToTaskFrame::setTask(int feature_id, bool reverse_direction)
{
  getFeatures();
  geometry_msgs::Pose[] task_frames = getTaskFrames(feature_id);

  if (!reverse_direction)
  {
    start_frame_.pose = task_frames[0];
    end_frame_.pose = task_frames[1];
    setProperty("goal", end_frame_);
  }
  else
  {
    start_frame_.pose = task_frames[1];
    end_frame_.pose = task_frames[0];
    setProperty("goal", start_frame_);
  }
  // Linear Feature
  if (task_frames.size() == 2)
  {
    if (planner_plugin == "ompl_interface/OMPLPlanner")
    {
      setLinConstraints()
    }
  }
  // Circular Feature
  else if (task_frames.size() == 3)
    // Spline Feature
    else if (task_frames.size() > 3)
}

// /// Set end Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
// /// technology model and additional offsets for e.g. approach/retrieve
// void MoveToTaskFrame::setTaskEnd(const std::string& feature_frame, const std::string& feature_to_task_frame,
//                                  const geometry_msgs::Transform offset_transform)
// {
//   end_frame_ = getTaskFrame(feature_frame, feature_to_task_frame, offset_transform);
//   setProperty("goal", end_frame_);
// }

// /// Set end Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask transformation
// void MoveToTaskFrame::setTaskEnd(const std::string& feature_frame,
//                                  const geometry_msgs::Transform feature_to_task_transform)
// {
//   identity_transform_.translation.x = 0;
//   identity_transform_.translation.y = 0;
//   identity_transform_.translation.z = 0;
//   identity_transform_.rotation.x = 0;
//   identity_transform_.rotation.y = 0;
//   identity_transform_.rotation.z = 0;
//   identity_transform_.rotation.z = 1;
//   end_frame_ = getTaskFrame(feature_frame, feature_to_task_transform, identity_transform_);
//   setProperty("goal", end_frame_);
// }

// /// Set start Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
// /// technology model and additional offsets for e.g. approach/retrieve
// void MoveToTaskFrame::setTaskStart(const std::string& feature_frame, const std::string& feature_to_task_frame,
//                                    const geometry_msgs::Transform offset_transform)
// {
//   std::string start_frame = feature_frame;
//   eraseSubStr(start_frame, "start");
//   eraseSubStr(start_frame, "center");
//   eraseSubStr(start_frame, "end");
//   start_frame += "start";
//   start_frame_ = getTaskFrame(start_frame, feature_to_task_frame, offset_transform);
// }

// /// Set start Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask transformation
// void MoveToTaskFrame::setTaskStart(const std::string& feature_frame,
//                                    const geometry_msgs::Transform feature_to_task_transform)
// {
//   identity_transform_.translation.x = 0;
//   identity_transform_.translation.y = 0;
//   identity_transform_.translation.z = 0;
//   identity_transform_.rotation.x = 0;
//   identity_transform_.rotation.y = 0;
//   identity_transform_.rotation.z = 0;
//   identity_transform_.rotation.z = 1;
//   std::string start_frame = feature_frame;
//   eraseSubStr(start_frame, "start");
//   eraseSubStr(start_frame, "center");
//   eraseSubStr(start_frame, "end");
//   start_frame += "start";
//   start_frame_ = getTaskFrame(start_frame, feature_to_task_transform, identity_transform_);
// }

// /// Set interim Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
// /// technology model and additional offsets for e.g. approach/retrieve
// void MoveToTaskFrame::setTaskInterim(const std::string& feature_frame, const std::string& feature_to_task_frame,
//                                      const geometry_msgs::Transform offset_transform)
// {
//   std::string interim_frame = feature_frame;
//   eraseSubStr(interim_frame, "start");
//   eraseSubStr(interim_frame, "end");
//   interim_frame += "center";
//   interim_frame_ = getTaskFrame(interim_frame, feature_to_task_frame, offset_transform);
// }

// /// Set interim Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask
// transformation void MoveToTaskFrame::setTaskInterim(const std::string& feature_frame,
//                                      const geometry_msgs::Transform feature_to_task_transform)
// {
//   identity_transform_.translation.x = 0;
//   identity_transform_.translation.y = 0;
//   identity_transform_.translation.z = 0;
//   identity_transform_.rotation.x = 0;
//   identity_transform_.rotation.y = 0;
//   identity_transform_.rotation.z = 0;
//   identity_transform_.rotation.z = 1;
//   std::string interim_frame = feature_frame;
//   eraseSubStr(interim_frame, "start");
//   eraseSubStr(interim_frame, "end");
//   interim_frame += "center";
//   interim_frame_ = getTaskFrame(interim_frame, feature_to_task_transform, identity_transform_);
// }

/// Set line constraints for OMPL Constrained Planning
moveit_msgs::Constraints MoveToTaskFrame::setLinConstraints()
{
  geometry_msgs::PoseStamped start_frame = start_frame_;
  geometry_msgs::PoseStamped goal_frame = end_frame_;
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

  geometry_msgs::Pose constraints_middle_frame;
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

    geometry_msgs::Pose constraints_middle_frame;
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

// void MoveToTaskFrame::eraseSubStr(std::string& mainStr, const std::string& toErase)
// {
//   // Search for the substring in string
//   size_t pos = mainStr.find(toErase);

//   if (pos != std::string::npos)
//   {
//     // If found then erase it from string
//     mainStr.erase(pos, toErase.length());
//   }
// }

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
