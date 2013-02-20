#include <ros/ros.h>

#include <Eigen/Eigen>

#include <jacobian.hpp>

#include <arm_navigation_msgs/GetRobotState.h>
#include <planning_environment/models/collision_models.h>

void
getRightArmJointPositions(const sensor_msgs::JointState& jstate,std::vector<double>* jvals)
{
  std::string group_name = "rarm_U_chest";
  
  planning_environment::CollisionModels collision_models("robot_description");
  std::vector<std::string> joint_names = collision_models.getKinematicModel()->getModelGroup(group_name)->getJointModelNames();
        
  jvals->resize(joint_names.size());
  
  for(std::vector<std::string>::const_iterator i=joint_names.begin(); i!=joint_names.end(); ++i)
  {
    for(std::vector<std::string>::const_iterator j=jstate.name.begin(); j!=jstate.name.end(); ++j)
    {
      if( !strcmp(i->c_str(),j->c_str()) )
      {
        jvals->at( i-joint_names.begin() ) = jstate.position.at( j-jstate.name.begin() );
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_tester");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  //get a jacobian
  Jacobian jac_utils;
  jac_utils.initialize();
  
  // Call to /environment_server/get_robot_state srv
  arm_navigation_msgs::GetRobotState::Request req;
  arm_navigation_msgs::GetRobotState::Response res;
  
  ros::service::waitForService("/environment_server/get_robot_state");
  ros::ServiceClient get_robot_state_client = nh.serviceClient<arm_navigation_msgs::GetRobotState>("/environment_server/get_robot_state");
  
  if ( !get_robot_state_client.call(req, res) )
  { 
    ROS_ERROR("A call get_robot_state srv, Init vertex: FAILED");
    return 1;
  }

   
//  std::vector<double> joint_states;
//  getRightArmJointPositions(res.robot_state.joint_state,&joint_states);
//  
//  Eigen::Map<Eigen::VectorXd> eigen_joint_states(&joint_states[0], joint_states.size());
//  
//  Eigen::MatrixXd jacobian;

//  jac_utils.getJacobian(eigen_joint_states, jacobian);

//  std::cout << "the jacobian is" << std::endl << jacobian << std::endl;
//  std::cout << "and the manipulability measure is: " << jac_utils.getManipulabilityMeasure(eigen_joint_states) << std::endl;

  return 0;
}
