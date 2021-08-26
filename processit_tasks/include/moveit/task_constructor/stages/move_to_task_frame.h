#pragma once
// #ifndef MOVE_TO_TASK_FRAME_H
// #define MOVE_TO_TASK_FRAME_H

#include <moveit/task_constructor/stages/move_to.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <processit_msgs/msg/weld_seam.h>
#include <processit_msgs/srv/load_task_description.h>

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
// using namespace moveit::task_constructor;

/** Perform a Motion to a task frame given a Feature Frame and FeatureToTask Transformation */
class MoveToTaskFrame : public MoveTo
{
public:
  MoveToTaskFrame(const std::string& name = "move to task frame",
                  const solvers::PlannerInterfacePtr& planner = solvers::PlannerInterfacePtr())
    : MoveTo(name, planner){};

  void startState(int feature_id, bool reverse_direction);

  void setTask(int feature_id, bool reverse_direction);

  //     /// Set end Pose of task as next MoveIt goal from TF Feature frame name
  //     void setTaskEnd(const std::string& feature_frame)
  // {
  //   setTaskEnd(feature_frame, workpiece_frame_, identity_transform_);
  // }

  // /// Set end Pose of task as next MoveIt goal from TF Feature frame name and FeatureToTask Frame from technology
  // model void setTaskEnd(const std::string& feature_frame, const std::string& feature_to_task_frame)
  // {
  //   setTaskEnd(feature_frame, feature_to_task_frame, identity_transform_);
  // }

  // /// Set end Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
  // /// technology model and additional offsets for e.g. approach/retrieve
  // void setTaskEnd(const std::string& feature_frame, const std::string& feature_to_task_frame,
  //                 const geometry_msgs::Transform offset_transform);

  // /// Set end Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask transformation
  // void setTaskEnd(const std::string& feature_frame, const geometry_msgs::Transform feature_to_task_transform);

  // /// Set start Pose of task as next MoveIt goal from TF Feature frame name
  // void setTaskStart(const std::string& feature_frame)
  // {
  //   setTaskStart(feature_frame, workpiece_frame_, identity_transform_);
  // }

  // /// Set start Pose of task as next MoveIt goal from TF Feature frame name and FeatureToTask Frame from
  // technology model void setTaskStart(const std::string& feature_frame, const std::string& feature_to_task_frame)
  // {
  //   setTaskStart(feature_frame, feature_to_task_frame, identity_transform_);
  // }

  // /// Set start Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
  // /// technology model and additional offsets for e.g. approach/retrieve
  // void setTaskStart(const std::string& feature_frame, const std::string& feature_to_task_frame,
  //                   const geometry_msgs::Transform offset_transform);

  // /// Set start Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask transformation
  // void setTaskStart(const std::string& feature_frame, const geometry_msgs::Transform feature_to_task_transform);

  // /// Set interim Pose of task as next MoveIt goal from TF Feature frame name
  // void setTaskInterim(const std::string& feature_frame)
  // {
  //   setTaskInterim(feature_frame, workpiece_frame_, identity_transform_);
  // }

  // /// Set interim Pose of task as next MoveIt goal from TF Feature frame name and FeatureToTask Frame from
  // technology model void setTaskInterim(const std::string& feature_frame, const std::string&
  // feature_to_task_frame)
  // {
  //   setTaskInterim(feature_frame, feature_to_task_frame, identity_transform_);
  // }

  // /// Set interim Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask Frame from
  // /// technology model and additional offsets for e.g. approach/retrieve
  // void setTaskInterim(const std::string& feature_frame, const std::string& feature_to_task_frame,
  //                     const geometry_msgs::Transform offset_transform);

  // /// Set interim Pose of task as next MoveIt goal with specified TF Feature frame name and FeatureToTask transformation
  // void setTaskInterim(const std::string& feature_frame, const geometry_msgs::Transform feature_to_task_transform);

  /// Set line constraints for OMPL Constrained Planning
  moveit_msgs::msg::Constraints setLinConstraints();

  /// get constraints for a given feature frame
  moveit_msgs::msg::Constraints setCircConstraints(const std::string& feature_frame);

private:
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
  processit_msgs::msg::WeldSeam[] current_features_;

  /// Get TF frame name of the workpiece containing the feature
  // bool getCurrentWorkpieceFrame(const std::string& feature_frame);

  /// Get FeatureToTask transform associated to a workpiece given TF FeatureToTask frame name from technology model
  // geometry_msgs::TransformStamped getFeatureToTaskTransform(const std::string& feature_to_task_frame);

  void getFeatures();

  geometry_msgs::msg::Pose[] getTaskFrames(int feature_id);

  //     /// Get pose of task frame with specified TF Feature frame name and TF FeatureToTask frame name from technology
  //     /// model and additional offsets for e.g. approach/retrieve
  //     geometry_msgs::PoseStamped
  //     getTaskFrame(const std::string& feature_frame, const std::string& feature_to_task_frame,
  //                  const geometry_msgs::Transform offset_transform);

  // /// Get pose of task frame with specified TF Feature frame name and FeatureToTask transformation and additional
  // /// offsets for e.g. approach/retrieve
  // geometry_msgs::PoseStamped getTaskFrame(const std::string& feature_frame,
  //                                         const geometry_msgs::Transform feature_to_task_transform,
  //                                         const geometry_msgs::Transform offset_transform);

protected:
  // Erase First Occurrence of given substring from main string.
  // void eraseSubStr(std::string& mainStr, const std::string& toErase);
};
// } // namespace processit_tasks
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
