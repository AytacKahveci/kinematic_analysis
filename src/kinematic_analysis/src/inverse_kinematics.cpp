#include "kinematic_analysis/inverse_kinematics.h"

namespace kinematics_solver
{
	InverseKinematicSolver::InverseKinematicSolver(const ros::NodeHandle& nh) : 
			nh_(nh)
	{
		if(!nh_.getParam("robot_description", robot_desc_string_))
		{
			throw std::runtime_error("Robot description parameter couldn't find in the parameter server!");
		}
		
		KDL::Tree tree;
		if(!kdl_parser::treeFromString(robot_desc_string_, tree))
		{
			throw std::runtime_error("Failed to construct KDL tree");
		}

		KDL::SegmentMap segments = tree.getSegments();

		for(auto segment = segments.begin(); segment != segments.end(); segment++)
		{
			std::cout << segment->first << std::endl;
		}

		const int num_joints = tree.getNrOfJoints();
		ROS_INFO("%d numbers of joints are used", num_joints);

		if(!tree.getChain("base_link", "link_6", chain_))
		{
			throw std::runtime_error("Could not get chain!");
		}

		// Set solver parameters
		int maxIterations=100000; // solver max loops
		double epsilon=0.01; // loop finish condition
		double lambda=0.05; // used in WDLS algorithm;

		auto Mx = Eigen::MatrixXd::Identity(6,1);
		ik_solver_vel_.reset(new KDL::ChainIkSolverVel_wdls(chain_, epsilon, maxIterations));
		ik_solver_vel_->setLambda(lambda);
		ik_solver_vel_->setWeightTS(Mx);
		fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
		ik_solver_.reset(new KDL::ChainIkSolverPos_NR(chain_, *fk_solver_, *ik_solver_vel_, maxIterations, epsilon));
	}

	bool InverseKinematicSolver::calcIK(const geometry_msgs::PoseStamped &request, 
																			std::vector<double> &result)
	{
		req_.M.Quaternion(request.pose.orientation.x,
											request.pose.orientation.y,
											request.pose.orientation.z,
											request.pose.orientation.w);
		req_.p = KDL::Vector(request.pose.position.x,
													request.pose.position.y,
													request.pose.position.z);

		result.clear();

		KDL::JntArray q_init(chain_.getNrOfJoints());
		for(int i = 0; i < chain_.getNrOfJoints(); i++)
		{
			q_init.data[i] = 0.0;
		}
		q_init.data << 1.0, -0.1, 0.3, 0.4, 1.0, -1.40;
		KDL::JntArray q_out(chain_.getNrOfJoints());
		
		int id = ik_solver_->CartToJnt(q_init, req_, q_out);
		if(id < 0)
		{
			ROS_ERROR("Error ik_solver:%d",id);
			return false;
		}

		ROS_INFO("Result of inverse kinematic:");
		for(int i = 0; i < chain_.getNrOfJoints(); i++)
		{
			std::cout << q_out.data[i] << std::endl;
			result.push_back(q_out.data[i]);
		}
	
		return true;
	}

} // namespace kinematics_solver



int main(int argc, char** argv)
{
	ros::init(argc, argv, "inverse_kinematics");

	ros::NodeHandle nh;

	kinematics_solver::InverseKinematicSolver ik(nh);

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
	ik.calcIK(target, result);
}