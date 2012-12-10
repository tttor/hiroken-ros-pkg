#include <ros/ros.h>

#include <arm_navigation_msgs/RobotTrajectory.h>

#include "hiro_sensor/Sense.h"
#include "planner_manager/Plan.h"
#include "hiro_control/MoveArm.h"

using namespace std;

class GeneralManager
{
public:
GeneralManager(ros::NodeHandle& nh)
  :nh_(nh)
{
  online_ = false;// Set the default
  if( !ros::param::get("/is_online", online_) )
    ROS_WARN("Can not get /is_online, use the default value (=false) instead");
    
  ros::service::waitForService("/sense_see");
  ros::service::waitForService("/plan");
  ros::service::waitForService("/move_arm");
  
  ROS_INFO("GeneralManager: UP and RUNNING ...");
  
  if( !online_ )
    goto_home();
}

~GeneralManager()
{ }

bool
sense()
{
  ros::service::waitForService("/sense_see");
    
  ros::ServiceClient sense_see_client;
  sense_see_client = nh_.serviceClient<hiro_sensor::Sense> ("/sense_see");
  
  hiro_sensor::Sense::Request sense_see_req;
  hiro_sensor::Sense::Response sense_see_res;
  
  sense_see_req.sensor_type = 1;
  
  if( !sense_see_client.call(sense_see_req, sense_see_res) ) 
  {
    ROS_WARN("Can't sense to see!");
    return false;
  }
  
  return true;
}

bool
plan()
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request plan_req;
  planner_manager::Plan::Response plan_res;
  
  if( !plan_client.call(plan_req, plan_res) ) 
  {
    ROS_WARN("Call plan srv: FAILED");
    return false;
  }
  
  return true;
}

//bool
//act()
//{

//}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;
//! Whether this is simulation or a real physical robot.
/*!
  More ...
*/
bool online_;

bool
goto_home()
{
  // Create a simple plan with 2 waypoints: start and goal
  arm_navigation_msgs::RobotTrajectory plan;
  
  plan.joint_trajectory.header.stamp = ros::Time::now();
  
  plan.joint_trajectory.joint_names.push_back("joint_chest_yaw");
  
  plan.joint_trajectory.joint_names.push_back("joint_rshoulder_yaw");
  plan.joint_trajectory.joint_names.push_back("joint_rshoulder_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_relbow_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_rwrist_yaw");
  plan.joint_trajectory.joint_names.push_back("joint_rwrist_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_rwrist_roll");
  
  plan.joint_trajectory.joint_names.push_back("joint_lshoulder_yaw");
  plan.joint_trajectory.joint_names.push_back("joint_lshoulder_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_lelbow_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_lwrist_yaw");
  plan.joint_trajectory.joint_names.push_back("joint_lwrist_pitch");
  plan.joint_trajectory.joint_names.push_back("joint_lwrist_roll");

  trajectory_msgs::JointTrajectoryPoint start;
  
  start.positions.push_back(0.);
  
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  start.positions.push_back(0.);
  
  plan.joint_trajectory.points.push_back(start);
  
  trajectory_msgs::JointTrajectoryPoint goal;
  
  goal.positions.push_back(0.);
  
  goal.positions.push_back(0.15707963267948966);  
  goal.positions.push_back(-2.7017696820872223);
  goal.positions.push_back(-2.6162485487395);
  goal.positions.push_back(2.0263272615654166);
  goal.positions.push_back(-0.38048177693476387);
  goal.positions.push_back(0.17453292519943295);
  
  goal.positions.push_back(0.15707963267948966);  
  goal.positions.push_back(-2.7017696820872223);
  goal.positions.push_back(-2.6162485487395);
  goal.positions.push_back(-2.0263272615654166);// "joint_lwrist_yaw"
  goal.positions.push_back(-0.38048177693476387);
  goal.positions.push_back(0.17453292519943295);

  plan.joint_trajectory.points.push_back(goal);
    
  // Sending commmands to joint_controller
  hiro_control::MoveArm::Request req;
  hiro_control::MoveArm::Response res;

  req.robot_trajectory = plan;
  
  ros::service::waitForService("/move_arm");  
  ros::ServiceClient move_arm_client;
  move_arm_client = nh_.serviceClient<hiro_control::MoveArm>("/move_arm");
  
  if ( !move_arm_client.call(req, res) )
  {
    ROS_ERROR("MoveArm service failed on %s",move_arm_client.getService().c_str());
    return false;
  }
  return true;
}
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "general_mgr");
  ros::NodeHandle nh;
  
  GeneralManager gm(nh);
  
  for(size_t i=0; i<3; ++i)// At least 3 times, otherwise the planner_manager will miss collission object publications.
    gm.sense();
    
  gm.plan();
  
//  gm.train();

//  gm.act();// TODO adjust hiro_joint_controller.cpp and hiro_joint_action_server.cpp to accept different jspaces
  
  ROS_INFO("general_mgr: spinning...");
  ros::spin();
  
  return 0;
}
