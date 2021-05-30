#include "kinematic_analysis/moveit_planner.h"

MoveitPlanner::MoveitPlanner(const std::string& planning_group)
  : planning_group_(planning_group)
{
  spinner_.reset(new ros::AsyncSpinner(1));
  spinner_->start();
  
  robot_model_loader_ = RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  robot_model_ = robot_model_loader_->getModel();

  move_group_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));

  planning_frame_ = move_group_->getPlanningFrame();
  eef_frame_ = move_group_->getEndEffectorLink();
  ROS_INFO("Planning Frame: %s", move_group_->getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group_->getEndEffectorLink().c_str());

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  ros::NodeHandle nh;
  
  // Get planning scene object
  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface(nh.getNamespace()));
  
  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planning_group_);
  robot_state_ = move_group_->getCurrentState();
}

MoveitPlanner::~MoveitPlanner()
{ }


bool MoveitPlanner::groupStateValidityFn(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group,
                          const double* joint_group_variable_values)
{
  return true;
}


bool MoveitPlanner::calcIK(const geometry_msgs::PoseStamped &request, 
                           std::vector<double> &result)
{
  bool stat = robot_state_->setFromIK(joint_model_group_, request.pose, move_group_->getEndEffectorLink(), 1, 10.0);
  
  if(stat)
  {
    robot_state_->copyJointGroupPositions(planning_group_, result);
    ROS_INFO("Inverse kinematics solution:");
    for(auto j : result)
      std::cout << j << std::endl;
  }
  else
		ROS_ERROR("Inverse kinematics solution could not found");

  return stat;
}


/* Get end-effector pose */
geometry_msgs::Pose MoveitPlanner::getState()
{
  geometry_msgs::PoseStamped eef_pose = move_group_->getCurrentPose();

  geometry_msgs::Pose pose = eef_pose.pose;

  return pose;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_planner");

  MoveitPlanner mvp("default_group");

  double px = -0.067;
	double py = -0.861;
	double pz = 0.332;
	double roll = -0.383;
	double pitch = 0.878;
	double yaw = 3.079;

	geometry_msgs::PoseStamped target;
	target.pose.position.x = px;
	target.pose.position.y = py;
	target.pose.position.z = pz;
	target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	std::cout << "Target pose: " << 
						"px: " 		<< px 	<< ", " <<
						"py: " 		<< py 	<< ", " <<
						"pz: " 		<< pz 	<< "\n" <<
						"roll: " 	<< roll << ", " <<
						"pitch: " << pitch << ", " <<
						"yaw: " 	<< yaw  << std::endl;

  std::vector<double> result;
	mvp.calcIK(target, result);
  return 0;
}