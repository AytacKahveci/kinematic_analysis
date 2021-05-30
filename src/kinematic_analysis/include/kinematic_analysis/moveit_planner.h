#ifndef MOVEIT_PLANNER_H_
#define MOVEIT_PLANNER_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/kinematic_options.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<robot_model_loader::RobotModelLoader> RobotModelLoaderPtr;
typedef boost::shared_ptr<planning_scene::PlanningScene> PlanningScenePtr;

/** \brief Trajectory Planner which uses moveit backend
 * 
*/
class MoveitPlanner
{
public:
  /** \brief Constructor
   * 
   * \param[in] planning_group: Moveit planning group name
  */
  MoveitPlanner(const std::string& planning_group);

  /** \brief Destructor
   * 
  */
  ~MoveitPlanner();

  /** \brief Calculate inverse kinematics
   * 
   *  \param[in] request: End-effector target pose
   *  \param[in] result: Joint angles
   *  
   *  \return True if inverse kinematic solution is found, False otherwise
  */
  bool calcIK(const geometry_msgs::PoseStamped &request, 
              std::vector<double> &result);

  /** \brief groupStateValidityFn
   * 
   *  Not implemented
  */
  bool groupStateValidityFn(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group,
                            const double* joint_group_variable_values);

  /** \brief Get current end-effector pose
   * 
   *  \return Current end-effector pose
  */
  geometry_msgs::Pose getState();

protected:
  MoveGroupPtr move_group_;
  RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelConstPtr robot_model_;

  PlanningScenePtr planning_scene_;
  // Planning scene interface pointer
  boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  std::string planning_group_;
  double CART_STEP_SIZE_ = 0.01;
  double CART_JUMP_THRESH_ = 0.0;
  bool AVOID_COLLISIONS_ = true;
  std::string planning_frame_;
  std::string eef_frame_;

  boost::shared_ptr<ros::AsyncSpinner> spinner_;
};

#endif