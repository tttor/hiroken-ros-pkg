#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "action_manager/Commit.h"
#include "action_manager/Go2Home.h"
#include "hiro_control/MoveArm.h"
#include "hiro_control/ControlHand.h"

#include <boost/graph/depth_first_search.hpp>
#include <boost/algorithm/string.hpp>
#include "tmm_utils.hpp"
#include "hiro_utils.hpp"
#include "utils.hpp"

static const double COL_OBJ_PUB_TIME = 1.0;
static const std::string SET_PLANNING_SCENE_DIFF_SRV_NAME = "/environment_server/set_planning_scene_diff";

class ActionManager: public boost::dfs_visitor<>
{
public:
ActionManager(ros::NodeHandle nh)
:nh_(nh)
{
  online_ = false;// Set the default
  if( !ros::param::get("/is_online", online_) )
    ROS_WARN("Can not get /is_online, use the default value (=false) instead");
    
  motion_plan_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("motion_plan", 1000);
  
  att_collision_object_pub_ = nh_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);
 
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);
  set_planning_scene_diff_client_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_SRV_NAME);
}

bool
commit_srv_handle(action_manager::Commit::Request& req, action_manager::Commit::Response& res)
{
  ROS_DEBUG("Committing man_plan: BEGIN");
  
  TaskMotionMultigraph sol_tmm;
  TMMVertex sol_tmm_root;
  
  read_sol(&sol_tmm,&sol_tmm_root);
  
  depth_first_visit( sol_tmm,sol_tmm_root,ActionManager(nh_),get(vertex_color,sol_tmm) );
  
  ROS_DEBUG("Committing man_plan: END"); 
  return true;
}

bool
go2home_srv_handle(action_manager::Go2Home::Request& req, action_manager::Go2Home::Response& res)
{
  return go2home();
}

template <class Graph>
void 
discover_vertex(typename boost::graph_traits<Graph>::vertex_descriptor v,Graph& g)
{
  ROS_DEBUG_STREAM("Discover v= " << get(vertex_name,g,v));
    
  // Intrepret the symbolic state in v
  std::string name = get(vertex_name,g,v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );
  
  std::string act_str = name_parts.at(0);
  
  if( !strcmp(act_str.c_str(),"MessyHome") or !strcmp(act_str.c_str(),"TidyHome") or !strcmp(act_str.substr(0,7).c_str(),"TmpHome"))
    return;
    
  std::vector<std::string> act_str_parts;
  boost::split( act_str_parts, act_str, boost::is_any_of("_") );
  
  std::string op = act_str_parts.at(0);
  std::string obj_id = act_str_parts.at(1);
  std::string rbt_id = act_str_parts.at(2);
  
  // Call grasp()
  if( !strcmp(op.c_str(),"Grasped") )
  {
    commit_grasp(obj_id,rbt_id);
  }
  else if( !strcmp(op.c_str(),"Released") )
  {
    commit_ungrasp(obj_id,rbt_id);
  }
}

template <class Graph>
void examine_edge(typename boost::graph_traits<Graph>::edge_descriptor e,Graph& g)
{
  ROS_DEBUG_STREAM("Examine e= " << get(edge_name,g,e));
  
  commit_motion(  utils::get_plan( get(edge_planstr,g,e) )   );
}

private:
bool
go2home()
{
 // Create a simple plan with 2 waypoints: start and goal
  trajectory_msgs::JointTrajectory plan;
  
  plan.header.stamp = ros::Time::now();
  
  plan.joint_names.push_back("joint_chest_yaw");
  
  plan.joint_names.push_back("joint_rshoulder_yaw");
  plan.joint_names.push_back("joint_rshoulder_pitch");
  plan.joint_names.push_back("joint_relbow_pitch");
  plan.joint_names.push_back("joint_rwrist_yaw");
  plan.joint_names.push_back("joint_rwrist_pitch");
  plan.joint_names.push_back("joint_rwrist_roll");
  
  plan.joint_names.push_back("joint_lshoulder_yaw");
  plan.joint_names.push_back("joint_lshoulder_pitch");
  plan.joint_names.push_back("joint_lelbow_pitch");
  plan.joint_names.push_back("joint_lwrist_yaw");
  plan.joint_names.push_back("joint_lwrist_pitch");
  plan.joint_names.push_back("joint_lwrist_roll");

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
  
  plan.points.push_back(start);
  
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

  plan.points.push_back(goal);
    
  // Sending commmands to joint_controller
  hiro_control::MoveArm::Request req;
  hiro_control::MoveArm::Response res;

  req.trajectory = plan;
  
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

bool
read_sol(TaskMotionMultigraph* sol_tmm,TMMVertex* sol_tmm_root)
{
  std::string data_path= ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  boost::dynamic_properties sol_tmm_dp;
  
  sol_tmm_dp.property("vertex_id", get(vertex_name, *sol_tmm));
  
  sol_tmm_dp.property( "label",get(edge_name, *sol_tmm) );
  sol_tmm_dp.property( "weight",get(edge_weight, *sol_tmm) );  
  sol_tmm_dp.property( "jspace",get(edge_jspace, *sol_tmm) );
  sol_tmm_dp.property( "color",get(edge_color, *sol_tmm) );
  sol_tmm_dp.property( "srcstate",get(edge_srcstate, *sol_tmm) );  
  sol_tmm_dp.property( "mptime",get(edge_mptime, *sol_tmm) );  
  sol_tmm_dp.property( "planstr",get(edge_planstr, *sol_tmm) );
      
  std::string sol_tmm_path = data_path + "/sol_tmm.dot";
  std::ifstream sol_tmm_dot(sol_tmm_path.c_str());
  read_graphviz(sol_tmm_dot, *sol_tmm, sol_tmm_dp, "vertex_id");  
  
  boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vj, vj_end;
  for(boost::tie(vj,vj_end) = vertices(*sol_tmm); vj!=vj_end; ++vj)
  {
    if( !strcmp(get(vertex_name,*sol_tmm,*vj).c_str(),"MessyHome") )
    {
      *sol_tmm_root = *vj;
      break;
    }
  }
  
  return true;
}

//! Commit a motion plan
/*!
  More...
  
  \param plan A motion plan, which is a trajectory (path)
  \return whether successful
*/
bool
commit_motion(const trajectory_msgs::JointTrajectory& plan)
{
  ROS_DEBUG("Commit a motion_plan: BEGIN");
  
  // Visualize the plan
  motion_plan_pub_.publish(plan);

  // Sendding commmands to joint_controller
  hiro_control::MoveArm::Request req;
  hiro_control::MoveArm::Response res;

  req.trajectory = plan;

  ros::service::waitForService("/move_arm");
  ros::ServiceClient commit_client = nh_.serviceClient<hiro_control::MoveArm>("/move_arm");
  if (commit_client.call(req, res))
    ROS_DEBUG("Move arm succeeded");
  else
  {
    ROS_ERROR("Move arm failed");
    return false;
  }
  
  // clear the motion plan visualization
  trajectory_msgs::JointTrajectory empty_path;
  motion_plan_pub_.publish(empty_path);
  
  ROS_DEBUG("Commit a motion_plan: END");
  return true;
}
//! Commit the grasp
/*!
  More...
  
  \param object The will-be-grasped object
  \return whether successful
*/
bool
commit_grasp(const std::string& obj_id,const std::string& rbt_id)
{
  std::string link_name;
  link_name = get_eof_link(rbt_id);
  
  arm_navigation_msgs::AttachedCollisionObject att_object;
  
  att_object.link_name = link_name;

  att_object.object.id = obj_id;
  att_object.object.header.frame_id = link_name;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;    
  
  att_collision_object_pub_.publish(att_object);
  ros::Duration(COL_OBJ_PUB_TIME).sleep();

  arm_navigation_msgs::SetPlanningSceneDiff::Request req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response res;  
    
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);
  if( !set_planning_scene_diff_client_.call(req,res) ) 
  {
    ROS_WARN("Can't get planning scene");
    return false;
  }
  
  if(online_) 
  {
    // Commit the real grasp
    hiro_control::ControlHand::Request control_hand_req;
    hiro_control::ControlHand::Response control_hand_res;
    
    control_hand_req.cmd = 1;
    
    ros::service::waitForService("control_hand");
    ros::ServiceClient control_hand_client;
    control_hand_client = nh_.serviceClient<hiro_control::ControlHand>("control_hand");
    
    if( !control_hand_client.call(control_hand_req, control_hand_res) ) 
    {
      ROS_ERROR("Can't call control_hand srv");
      return false;
    }
  }
  else
  {
    ros::Duration(.5).sleep();
  }
  
  ROS_DEBUG_STREAM("Have grasped= " << obj_id);
  return true;  
}
//! Commit the ungrasp
/*!
  More...
  
  \param object The will-be-released object
  \return whether successful
*/
bool
commit_ungrasp(const std::string& obj_id,const std::string& rbt_id)
{
  std::string link_name;
  link_name = get_eof_link(rbt_id);
  
  arm_navigation_msgs::AttachedCollisionObject att_object;
  
  att_object.link_name = link_name;

  att_object.object.id = obj_id;
  att_object.object.header.frame_id = link_name;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;    
  
  att_collision_object_pub_.publish(att_object);
  ros::Duration(COL_OBJ_PUB_TIME).sleep();

  arm_navigation_msgs::SetPlanningSceneDiff::Request req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response res;  
    
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);
  if(!set_planning_scene_diff_client_.call(req,res)) 
  {
    ROS_WARN("Can't get planning scene");
    return false;
  }
  
  if(online_)
  {
    // Commit the real ungrasp
    hiro_control::ControlHand::Request control_hand_req;
    hiro_control::ControlHand::Response control_hand_res;
    
    control_hand_req.cmd = 2;
    
    ros::service::waitForService("control_hand");
    ros::ServiceClient control_hand_client;
    control_hand_client = nh_.serviceClient<hiro_control::ControlHand>("control_hand");
    
    if( !control_hand_client.call(control_hand_req, control_hand_res) ) 
    {
      ROS_WARN("Can't call control_hand srv");
      return false;
    }
  }
  else
  {
    ros::Duration(.5).sleep();
  }
  
  ROS_DEBUG("Have been ungrasped");
  return true;  
}
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;
//! A publisher for visualizing a motion plan
/*!
  A publisher should be initialized at the contructor (?)
*/
ros::Publisher motion_plan_pub_;
//! A publisher for attached collision objects
/*!
  A publisher should be initialized at the contructor (?)
*/
ros::Publisher att_collision_object_pub_;
//! A client that is used at several places.
/*!
  More...
*/
ros::ServiceClient set_planning_scene_diff_client_;

bool online_;
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "action_mgr");
  ros::NodeHandle nh;
  
//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  ActionManager actor(nh);

  ros::ServiceServer commit_srv;
  commit_srv = nh.advertiseService("/commit", &ActionManager::commit_srv_handle, &actor);

  ros::ServiceServer go2home_srv;
  go2home_srv = nh.advertiseService("/go2home", &ActionManager::go2home_srv_handle, &actor);
  
  ROS_INFO("action_mgr: spinning...");
  ros::spin();
  
  return 0;
}
