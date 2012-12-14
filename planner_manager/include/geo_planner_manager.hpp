#ifndef GEO_PLANNER_MANAGER_HPP_INCLUDED
#define GEO_PLANNER_MANAGER_HPP_INCLUDED

#include <tf/transform_listener.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <boost/algorithm/string/erase.hpp>

#include "planner_manager.hpp"
#include "tmm_utils.hpp"

static const double COL_OBJ_PUB_TIME = 1.0;
static const std::string SET_PLANNING_SCENE_DIFF_SRV_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_PLANNING_SCENE_SRV_NAME = "/environment_server/get_planning_scene";
static const std::string GET_ROBOT_STATE_SRV_NAME = "/environment_server/get_robot_state";
static const std::string TRAJECTORY_FILTER_SRV_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

static const size_t NUM_PLANNING_ATTEMPTS = 1;
static const double ALLOWED_PLANNING_TIME = 1. * 60.;
static const double ALLOWED_SMOOTHING_TIME = 2.0;
static const size_t MAX_JSPACE_DIM = 7;

struct GeoPlanningCost
{
  GeoPlanningCost()
  :process(0.), result(0.)
  { }
  
  double process;
  double result;
  
  double w_p;
  double w_r;
  
  double
  total(const double& w)
  {
    return ( (w*process) + ((1.0-w)*result) );
  }
};

class GeometricPlannerManager
{
public:
GeometricPlannerManager(PlannerManager* pm)
: pm_(pm)
{ 
  collision_object_pub_ = pm_->nh_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  att_collision_object_pub_ = pm_->nh_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  motion_plan_pub_ = pm_->nh_.advertise<trajectory_msgs::JointTrajectory>("motion_plan", 1000);    
  
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);
  set_planning_scene_diff_client_ = pm_->nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_SRV_NAME);
}

bool
plan(TMMEdge e)
{
  // Get the start point /subseteq vertex_jstates (source of e)
  sensor_msgs::JointState start_state;
  start_state = get_jstate_subset(  get(edge_jspace, pm_->tmm_, e), get(vertex_jstates, pm_->tmm_, source(e, pm_->tmm_))  );
  ROS_DEBUG("start_state: SET");
  
  // Determine all possible goal poses using grasp(if GRASPED_) or ungrasp(if RELEASED_) planner
  std::vector<sensor_msgs::JointState> goal_set;// Note that we do not use std::set because of ros an msg constraint that is an array[] is handled as a vector
  if(target(e,pm_->tmm_)==pm_->tmm_goal_)
  {
    // Get a subset of initial jstates stored in the goal vertex, which is the home pose
    goal_set.push_back(   get_jstate_subset(  get(edge_jspace, pm_->tmm_, e), get(vertex_jstates, pm_->tmm_, target(e, pm_->tmm_))  )   );
  }
  else
  {
    // Call grasp/ungrasp planner here!
    goal_set = get_goal_set( target(e,pm_->tmm_),get(edge_jspace,pm_->tmm_,e) );  
  }
  ROS_DEBUG_STREAM("goal_set.size()= " << goal_set.size());
  
  // Prepare the planning environment to do motion planning
  set_planning_env( source(e,pm_->tmm_), true );
  
  // Put the geometric config of source vertex set above into edge prop.
  put( edge_srcstate,pm_->tmm_,e,get_planning_env() );
  
  // Obtain the best motion plan
  ROS_DEBUG("Start planning motions");
  trajectory_msgs::JointTrajectory best_plan;
  
  GeoPlanningCost cheapest_cost;// Initialize with a high value because it serves as a base for comparison
  cheapest_cost.result = std::numeric_limits<double>::max();
  cheapest_cost.process = std::numeric_limits<double>::max();

  // For calculating iter_cost
  size_t n_success = 0;
  size_t n_failure = 0;
  
  // Try to plan a motion for all possible goal states in the goal set. Then, take the best among them.
  // This means that we consider a goal set not only a single goal state.
  for(std::vector<sensor_msgs::JointState>::const_iterator i=goal_set.begin(); i!=goal_set.end(); ++i)
  {
    ROS_INFO_STREAM( "Attempt: " << i-goal_set.begin()+1 << "-th of " << goal_set.size() );

    trajectory_msgs::JointTrajectory plan;
    GeoPlanningCost cost;
    bool found;

    if(  plan_motion( start_state,*i,get(edge_jspace,pm_->tmm_,e),&plan,&cost,&found )  )
    {
      if(found)
      {
        ++n_success;
        ROS_DEBUG_STREAM("cost.total(0.5)= " << cost.total(0.5));
        
        if( cost.total(0.5) < cheapest_cost.total(0.5) )
        {
          ROS_DEBUG("The new plan is better");
          
          cheapest_cost = cost;
          best_plan = plan;
        }
      } 
      else
      {
        ++n_failure;
      }
    }
    else
    {
      ROS_WARN("A call to /ompl_planning/plan_kinematic_path and or /benchmark_motion_plan: FAILED.");
    }
  }
  ROS_INFO_STREAM("n_success= " << n_success);
  ROS_INFO_STREAM("n_failure= " << n_failure);
 
  if(n_success==0) 
  {
    ROS_INFO_STREAM("No motion plan for " << goal_set.size() << " goals in the goal set ");
    ROS_INFO_STREAM("OF e= " << get(edge_name,pm_->tmm_,e) << " in " << get(edge_jspace,pm_->tmm_,e));
        
    cheapest_cost.result = 0.;
    cheapest_cost.process = (ALLOWED_PLANNING_TIME*NUM_PLANNING_ATTEMPTS) + ALLOWED_SMOOTHING_TIME + exp(MAX_JSPACE_DIM/MAX_JSPACE_DIM);
  }
  
  // Calculate the cost of iterating over the goal set
  double iter_cost;
  if( goal_set.empty() )
    iter_cost = 0.;
  else
   iter_cost = exp( (double)n_failure/(double)(n_success+n_failure) );
  
  cheapest_cost.process += iter_cost;

  // Reset the planning environment
  reset_planning_env();

  // Put the best motion plan of this edge e and its geo. planning cost
  put(edge_plan, pm_->tmm_, e, best_plan);
  put(edge_planstr, pm_->tmm_,e,get_planstr(best_plan));
  put(edge_weight, pm_->tmm_, e, cheapest_cost.total(0.8));// Thus, process_cost is 4 times worth it than result_cost; 0.8:0.2
  put(edge_color, pm_->tmm_, e, std::string("red"));
  
  ROS_DEBUG_STREAM("Geo. planning for " << get(edge_name,pm_->tmm_,e) << " in " << get(edge_jspace,pm_->tmm_,e) << " ends successfully.");
  return true;
}
//! Convert plan representations from trajectory_msgs::JointTrajectory to std::string
/*!
  ...
*/
std::string
get_planstr(const trajectory_msgs::JointTrajectory& plan )
{
  std::string planstr;
  
  for(std::vector<std::string>::const_iterator i=plan.joint_names.begin(); i!=plan.joint_names.end(); ++i)
  {
    planstr += *i;
    planstr += ",";
  }
  boost::erase_last(planstr,",");
  planstr += ";";
  
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator i=plan.points.begin(); i!=plan.points.end(); ++i)
  {
    for(std::vector<double>::const_iterator j=i->positions.begin(); j!=i->positions.end(); ++j)
    {
      planstr += boost::lexical_cast<std::string>(*j);
      planstr += ",";
    }
    boost::erase_last(planstr,",");
    planstr += ";";
  }
  boost::erase_last(planstr,";");
  
  return planstr;
}
//! Set joint states of adjacent vertex of the given vertex v
/*!
  ...
*/
bool
set_av_jstates(TMMVertex v)
{
  ROS_DEBUG_STREAM("set_av_jstates for v= " << get(vertex_name,pm_->tmm_,v) << ": Begin.");
  
  // Determine the cheapest in_edge of the adjacent vertices of v
  std::map<TMMVertex,TMMEdge> av_ce_map;// av = adjacent_vertices to ce = cheapest_edge map
  
  graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
  for(tie(oei,oei_end) = out_edges(v, pm_->tmm_); oei!=oei_end; ++oei)
  {
    std::map<TMMVertex, TMMEdge>::iterator it;
    bool inserted;
    
    tie(it, inserted) = av_ce_map.insert( std::make_pair(target(*oei, pm_->tmm_),*oei) );
    
    if(  (!inserted) and ( get(edge_weight, pm_->tmm_, *oei) < get(edge_weight, pm_->tmm_, it->second) )  )
    {
      it->second = *oei;
    }
  }
  
  // Update the adjacent vertices's jstates using av_ce_map
  ROS_DEBUG("Updating av's jstates");
  
  graph_traits<TaskMotionMultigraph>::adjacency_iterator avi, avi_end;
  for(tie(avi,avi_end)=adjacent_vertices(v, pm_->tmm_); avi!=avi_end; ++avi )
  {
    sensor_msgs::JointState joint_state;
    joint_state = get(vertex_jstates, pm_->tmm_, *avi);
    
    // The updating_joint_state is at the last point of the mp of an edge
    trajectory_msgs::JointTrajectory motion_plan;
    motion_plan = get(edge_plan, pm_->tmm_, av_ce_map[*avi]);
    
    if(motion_plan.points.size()==0)
      continue;
      
    trajectory_msgs::JointTrajectoryPoint goal_point = motion_plan.points.back();
    
    for(std::vector<std::string>::const_iterator i=motion_plan.joint_names.begin(); i!=motion_plan.joint_names.end(); ++i)
    {
      std::vector<std::string>::iterator j;
      j = std::find(joint_state.name.begin(), joint_state.name.end(), *i);
      
      joint_state.position.at(j-joint_state.name.begin()) = goal_point.positions.at(i-motion_plan.joint_names.begin());
    }
    
    put(vertex_jstates, pm_->tmm_, *avi, joint_state);
  }
  
  ROS_DEBUG_STREAM("set_av_jstates for v= " << get(vertex_name,pm_->tmm_,v) << ": End.");
  return true;
}
  
void
mark_vertex(const TMMVertex& v)
{
  put(vertex_color, pm_->tmm_, v, color_traits<boost::default_color_type>::black());// for examined vertex
}

std::string
get_planning_env()
{
  std::string state_str;
  
  // Call to /environment_server/get_planning_scene srv
  arm_navigation_msgs::GetPlanningScene::Request req;
  arm_navigation_msgs::GetPlanningScene::Response res;
  
  ros::service::waitForService(GET_PLANNING_SCENE_SRV_NAME);
  ros::ServiceClient get_planning_scene_client = pm_->nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SRV_NAME);
  
  if ( !get_planning_scene_client.call(req, res) )
  { 
    ROS_ERROR("Failed to call get_planning_scene srv");
    return state_str;
  }
  
  // Put movable object poses
  std::vector<arm_navigation_msgs::CollisionObject> col_objs;
  col_objs = res.planning_scene.collision_objects;
  
  for(std::vector<arm_navigation_msgs::CollisionObject>::iterator i=col_objs.begin(); i!=col_objs.end(); ++i)
  {
    if( !strcmp(i->id.c_str(),"table") or !strcmp(i->id.c_str(),"vase") )
      continue;
    
    state_str = state_str + i->id + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).position.x) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).position.y) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).position.z) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).orientation.x) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).orientation.y) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).orientation.z) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(i->poses.at(0).orientation.w) + ";";// The last one use ";", instead of ","
  }
  
  // If the state is GRASPEDXXX, the object is already attached, so check for att_collision_object_pub_
  std::vector<arm_navigation_msgs::AttachedCollisionObject> att_col_objs;
  att_col_objs = res.planning_scene.attached_collision_objects;
  
  for(std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator i=att_col_objs.begin(); i!=att_col_objs.end(); ++i)
  {
    // AttachedCollisionObject automatically has its link_name as the reference frame.
    // Therefore, we have to lookup the transfrom from the object to /table (our common ref. frame for all movable object)
    // $ rosrun tf tf_echo /table CAN1

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
      listener.waitForTransform("/table", i->object.id, ros::Time::now(), ros::Duration(3.0));
      listener.lookupTransform("/table", i->object.id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    state_str = state_str + i->object.id + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getOrigin().x()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getOrigin().y()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getOrigin().z()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getRotation().x()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getRotation().y()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getRotation().z()) + ",";
    state_str = state_str + boost::lexical_cast<std::string>(transform.getRotation().w()) + ";";// The last one use ";", instead of ","
  }
  
  // TODO Put joint states
  sensor_msgs::JointState joint_state;
  joint_state = res.planning_scene.robot_state.joint_state;
  
  for(std::vector<std::string>::const_iterator i=joint_state.name.begin(); i!=joint_state.name.end(); ++i)
  {
    state_str += *i + ",";
    state_str += boost::lexical_cast<std::string>( joint_state.position.at(i-joint_state.name.begin()) ) + ";";
  }
  boost::erase_last(state_str,";");
  
  return state_str;
} 

void
init_vertex(const TMMVertex& v)
{
  // Call to /environment_server/get_robot_state srv
  arm_navigation_msgs::GetRobotState::Request req;
  arm_navigation_msgs::GetRobotState::Response res;
  
  ros::service::waitForService(GET_ROBOT_STATE_SRV_NAME);
  ros::ServiceClient get_robot_state_client = pm_->nh_.serviceClient<arm_navigation_msgs::GetRobotState>(GET_ROBOT_STATE_SRV_NAME);
  
  if ( !get_robot_state_client.call(req, res) )
  { 
    ROS_ERROR("A call get_robot_state srv, Init vertex: FAILED");
    return;
  }
  
  put(vertex_jstates, pm_->tmm_, v, res.robot_state.joint_state);
}

private:
sensor_msgs::JointState
get_jstate_subset(const std::string& jspace, sensor_msgs::JointState& jstates)
{
  sensor_msgs::JointState subset_jstates;
  
  // Interpret jspace names
  if( !strcmp(jspace.c_str(), "rarm_U_chest") )
  {
    subset_jstates.name.push_back("joint_chest_yaw");
    
    subset_jstates.name.push_back("joint_rshoulder_yaw");
    subset_jstates.name.push_back("joint_rshoulder_pitch");
    subset_jstates.name.push_back("joint_relbow_pitch");
    subset_jstates.name.push_back("joint_rwrist_yaw");
    subset_jstates.name.push_back("joint_rwrist_pitch");
    subset_jstates.name.push_back("joint_rwrist_roll");
  }
  else if( !strcmp(jspace.c_str(), "rarm") )
  {
    subset_jstates.name.push_back("joint_rshoulder_yaw");
    subset_jstates.name.push_back("joint_rshoulder_pitch");
    subset_jstates.name.push_back("joint_relbow_pitch");
    subset_jstates.name.push_back("joint_rwrist_yaw");
    subset_jstates.name.push_back("joint_rwrist_pitch");
    subset_jstates.name.push_back("joint_rwrist_roll");
  }
  
  subset_jstates.position.resize(subset_jstates.name.size());
  
  // Obtain the position-value subset of jstates
  for(std::vector<std::string>::iterator i=subset_jstates.name.begin(); i!=subset_jstates.name.end(); ++i)
  {
    for(std::vector<std::string>::const_iterator k=jstates.name.begin(); k!=jstates.name.end(); ++k)
    {
      if( !strcmp(i->c_str(),k->c_str()) )
        subset_jstates.position.at(i-subset_jstates.name.begin()) = jstates.position.at(k-jstates.name.begin());
    }
  }

  return subset_jstates;
}

std::vector<sensor_msgs::JointState>
get_goal_set(const TMMVertex& v, const std::string& jspace)
{
  std::vector<sensor_msgs::JointState> goal_set;// Because plan_grasp receive a vector, instead of a set
  
  // Set the planning environment (for grasp planning) so as to satisfy the init_state, i.e. by setting already-tidied-up object at tidy spot.
  set_planning_env(v);
      
  // Call grasp planner or ungrasp planner depending on out_the name of v
  std::string name = get(vertex_name, pm_->tmm_, v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );// e.g from "Grasped_CAN1[CAN2.CAN3.]" to "Grasped_CAN1" and "CAN2.CAN3.]"
  
  if(name_parts.size() < 2)// e.g. "TidyHome"
  {
    ROS_WARN("get_goal_set() returns an empty goal_set.");
    return goal_set;
  }
  
  std::vector<std::string> name_parts_0;
  boost::split( name_parts_0, name_parts.at(0), boost::is_any_of("_") );// e.g from "Grasped_CAN1" to "Grasped" and "CAN1"

  std::string state_name = name_parts_0.at(0);
  std::string object_id = name_parts_0.at(1);
  
  if( !strcmp(state_name.c_str(), "Grasped") )// Assume that GRASP is always in messy_spot
  {
    arm_navigation_msgs::CollisionObject object;
    object = pm_->messy_cfg_[object_id];
  
    plan_grasp(object, jspace, &goal_set);
  }
  else if( !strcmp(state_name.c_str(), "Released") )// Assume that UNGRASP is always in tidy_spot
  {
    arm_navigation_msgs::CollisionObject object;
    object = pm_->tidy_cfg_[object_id];
    
    plan_ungrasp(object, jspace, &goal_set);
  }
  
  // Reset the planning environment
  reset_planning_env();
  
  return goal_set;
}
//! Brief ...
/*!
  This is for both grasp operations.
  The plan_ungrasp also calls this function.
  
  \param &object The collision object to be grasp or ungrasped.
  \param *grasp_poses All possible grasp poses.
  \param *cost Geometric planning cost.
  \return whether successful.
*/
bool
plan_grasp(arm_navigation_msgs::CollisionObject& object, const std::string& jspace, std::vector<sensor_msgs::JointState>* grasp_poses)
{
  ROS_DEBUG("Grasp planning: BEGIN");
  
  // Call the plan_grasp srv
  grasp_planner::PlanGrasp::Request plan_grasp_req;
  grasp_planner::PlanGrasp::Response plan_grasp_res; 
  
  plan_grasp_req.object = object;
  plan_grasp_req.jspace = jspace;
  
  ros::service::waitForService("/plan_grasp");
  ros::ServiceClient plan_grasp_client;
  plan_grasp_client = pm_->nh_.serviceClient<grasp_planner::PlanGrasp>("/plan_grasp");
    
  if ( plan_grasp_client.call(plan_grasp_req, plan_grasp_res) )
  {
    ROS_DEBUG("Succeeded to call plan_grasp service");
    
    if( plan_grasp_res.grasp_plans.empty() )
      ROS_WARN("Although done successfully, but _no_ grasp plan.");
    
    *grasp_poses = plan_grasp_res.grasp_plans;// This may be empty
    
    /*
    Note that the geometric planning process cost returned by plan_grasp_srv is ignored for now for simplicity.
    If we would like to take account of it, we also have to define the result cost of the grasp pose that results in the best motion plan.
    */

    return true;
  }
  else
  {
    ROS_WARN("A call to plan_grasp service: FAILED");
    return false;
  }
}
//! Brief ...
/*!
  This is for ungrasp operations.
  This function calls plan_ungrasp().
  
  \param &object The collision object to be grasp or ungrasped.
  \param *grasp_poses All possible grasp poses.
  \param *cost Geometric planning cost.
  \return whether successful.
*/
bool
plan_ungrasp(arm_navigation_msgs::CollisionObject& object, const std::string& jspace, std::vector<sensor_msgs::JointState>* ungrasp_poses)
{
  // Set the object in the tidy_cfg in the planning_environment
  object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
  collision_object_pub_.publish(object);
  ros::Duration(COL_OBJ_PUB_TIME).sleep();

  if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    
  return plan_grasp(object, jspace, ungrasp_poses); 
}
//! Brief ...
/*!
  Assuming that the motion planner is an oracle i.e. always returns the best motion plan for given a pair of start and goal states.
  This returns false in 2 cases: a call to motion plan is failed and or a call to benchmark_motion_plan srv is failed. In both cases, any result is erronous.
  From Sucan PhD Thesis: "In general, the dimensionality of the space to plan in and the amount of time spent planning seem to be the more important parameters when estimating edge costs."
  
  (1) the planner does not put any values at motion_plan.joint_trajectory.points.at(0).velocities. Therefore, motion_plan.joint_trajectory.points.at(0).velocities.size() is always zero (by default) here.
  (2) The result cost of this motion plan is only determined by the path length. Other possibilities: path smothness, clearance
  
  \param start_state
  \param goal_state
  \param *path
  \param *cost
  \return whether successful
*/
bool
plan_motion(const sensor_msgs::JointState& start_state, const sensor_msgs::JointState& goal_state, const std::string& jspace, trajectory_msgs::JointTrajectory* plan, GeoPlanningCost* cost, bool* found)
{
  arm_navigation_msgs::GetMotionPlan::Request req;  
  arm_navigation_msgs::GetMotionPlan::Response res;
  
  req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
  req.motion_plan_request.group_name = jspace;
  req.motion_plan_request.num_planning_attempts = NUM_PLANNING_ATTEMPTS;
  req.motion_plan_request.allowed_planning_time = ros::Duration(ALLOWED_PLANNING_TIME);
  req.motion_plan_request.planner_id= std::string("");
  
  req.motion_plan_request.goal_constraints.joint_constraints.resize( goal_state.name.size() );
  for(size_t i=0; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = goal_state.name.at(i);
    req.motion_plan_request.goal_constraints.joint_constraints[i].position = goal_state.position.at(i);
    req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    req.motion_plan_request.goal_constraints.joint_constraints[i].weight = 1.;
  }  
  
  req.motion_plan_request.start_state.joint_state.name.resize(start_state.name.size());
  req.motion_plan_request.start_state.joint_state.position.resize(start_state.name.size());
  req.motion_plan_request.start_state.joint_state.velocity.resize(start_state.name.size());
  req.motion_plan_request.start_state.joint_state.effort.resize(start_state.name.size());
  
  for(size_t i=0; i < req.motion_plan_request.start_state.joint_state.name.size(); ++i)
  {
    req.motion_plan_request.start_state.joint_state.name.at(i) = start_state.name.at(i);
    req.motion_plan_request.start_state.joint_state.position.at(i) = start_state.position.at(i);
    req.motion_plan_request.start_state.joint_state.velocity.at(i) = 0.;
    req.motion_plan_request.start_state.joint_state.effort.at(i) = 0.;
  }  
  
  ros::service::waitForService("ompl_planning/plan_kinematic_path");
  ros::ServiceClient planning_client = pm_->nh_.serviceClient<arm_navigation_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");
  
  ros::Time planning_begin = ros::Time::now();
  if ( planning_client.call(req, res) )
  {
    double planning_time;
    planning_time = (ros::Time::now()-planning_begin).toSec();
    
    if ( res.trajectory.joint_trajectory.points.empty() )
    {
      ROS_INFO_STREAM("Motion planner was unable to plan a path to goal, joint_trajectory.points.size()= " << res.trajectory.joint_trajectory.points.size());
      *found = false;
    }
    else
    {
      ROS_INFO("Motion planning succeeded");
      *found = true;
      
      // Calculate jspace_cost
      size_t jspace_dim;
      jspace_dim = start_state.name.size(); // _must_ be equal to goal_state.name.size()
      
      double jspace_cost;
      jspace_cost = exp( (double)jspace_dim/(double)MAX_JSPACE_DIM );
      
      // Filter(smooth) the raw motion plan
      trajectory_msgs::JointTrajectory filtered_trajectory;
      ros::Time smoothing_begin = ros::Time::now();

      filter_path(res.trajectory.joint_trajectory, req, &filtered_trajectory);

      double smoothing_time;
      smoothing_time = (ros::Time::now()-smoothing_begin).toSec();
            
      // Visualize the path
      motion_plan_pub_.publish(filtered_trajectory);

      // Benchmark the filtered path to determine the result cost
      ros::service::waitForService("/benchmark_motion_plan");
      ros::ServiceClient benchmark_path_client = pm_->nh_.serviceClient<hiro_common::BenchmarkPath>("/benchmark_motion_plan");
      
      hiro_common::BenchmarkPath::Request benchmark_path_req;
      hiro_common::BenchmarkPath::Response benchmark_path_res;

      benchmark_path_req.trajectory = filtered_trajectory;

      if (benchmark_path_client.call(benchmark_path_req, benchmark_path_res))
      {
        ROS_DEBUG("BenchmarkPath succeeded");
      }
      else
      {
        ROS_ERROR("BenchmarkPath service failed on %s",benchmark_path_client.getService().c_str());
        return false;
      }
  
      // Accumulate the cost 
      cost->result = benchmark_path_res.length;
      cost->process = planning_time + smoothing_time + jspace_cost;
      
      // Put the resulted plan
      *plan = filtered_trajectory;
      
      // Clear the path visualization
      trajectory_msgs::JointTrajectory empty_path;
      ros::Duration(0.1).sleep();// May be not necessary, just to make the visualization time as longer as you want.
      motion_plan_pub_.publish(empty_path);
    }
    return true;
  }
  else
  {
    ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
    return false;// Although not truly appropriate, this must indicate that the call to this function is unsuccessful.
  }
}
//! Filter the motion path
/*!
  Note that whenever filter_path() returns false, it means that the trajectory pass in is not changed

  \param trajectory_in 
  \param ori_req
  \param *trajectory_out  
  \return whether successful
*/
bool
filter_path(const trajectory_msgs::JointTrajectory& trajectory_in, const arm_navigation_msgs::GetMotionPlan::Request& ori_req, trajectory_msgs::JointTrajectory* trajectory_out)
{
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;

//  req.trajectory.joint_names = trajectory_in.joint_names;
//  req.trajectory.points = trajectory_in.points;
  req.trajectory = trajectory_in;
  req.group_name = ori_req.motion_plan_request.group_name;
  req.start_state = ori_req.motion_plan_request.start_state;
  req.path_constraints = ori_req.motion_plan_request.path_constraints;
  req.goal_constraints = ori_req.motion_plan_request.goal_constraints;
  req.allowed_time = ros::Duration(ALLOWED_SMOOTHING_TIME);

  ros::service::waitForService(TRAJECTORY_FILTER_SRV_NAME);
  ros::ServiceClient filter_trajectory_client = pm_->nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SRV_NAME);
  
  if(filter_trajectory_client.call(req,res))
  {
    *trajectory_out = res.trajectory;
    
    if(trajectory_out->points.empty())
    {
      ROS_WARN("filter srv returns TRUE but it is empty; trajectory_out = trajectory_in");
      *trajectory_out = trajectory_in;
    }
    
    return true;
  }
  else
  {
    ROS_WARN("Service call to filter trajectory failed; trajectory_out = trajectory_in");

    *trajectory_out = trajectory_in;
    return false;
  }
}
//! Reset the collision space or planning environment.
/*!
  That is to the messy config.
  
  \return whether successful
*/
bool
reset_planning_env()
{
  for(std::map<std::string, arm_navigation_msgs::CollisionObject>::iterator i=pm_->messy_cfg_.begin(); i!=pm_->messy_cfg_.end(); ++i)
  {
    
    i->second.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    collision_object_pub_.publish( i->second );
    ros::Duration(COL_OBJ_PUB_TIME).sleep();
    
    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    {
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
      return false;
    }
    
    // Note that this brutal eset leads to: e.g.
    //[WARN] No attached body CAN1 attached to link link_rhand_palm
    //Sanity check failing - no entry in acm for collision space object CAN1
    //No entry in default collision matrix for attached body CAN1 when there really should be.
    //Must already have an entry in allowed collision matrix for CAN1
    
    arm_navigation_msgs::AttachedCollisionObject att_object;
  
    att_object.link_name = "link_rhand_palm";
    //att_object.touch_links.push_back("r_gripper_palm_link");
    att_object.object.id = i->first;
    att_object.object.header.frame_id = "link_rhand_palm";
    att_object.object.header.stamp = ros::Time::now();
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
    
    att_collision_object_pub_.publish(att_object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();
    
    ROS_INFO("waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME)");
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);

    if(!set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_)) {
      ROS_WARN("Can't get planning scene");
      return false;
    }
  }
  
  ROS_DEBUG("All movable objects should be back to their initial messy poses");
  return true;
}

bool
set_planning_env(const TMMVertex& v, const bool& for_motion_planning=false)
{
  //==========
  // This part only sets tidied objects (everything inside ...[]) to tidied positions
  std::string name = get(vertex_name, pm_->tmm_, v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );// e.g from "GRASPED_CAN1[CAN2.CAN3.]" to "GRASPED_CAN1" and "CAN2.CAN3.]"
  
  if(name_parts.size() < 2)// for e.g. "MessyHome"
    return true;
  
  std::vector<std::string> tidied_object_ids;
  boost::split( tidied_object_ids, name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to CAN2 and CAN3 and ]
  tidied_object_ids.erase(tidied_object_ids.end()-1);//remove a "]"

  for(std::vector<std::string>::const_iterator i=tidied_object_ids.begin(); i!=tidied_object_ids.end(); ++i)
  {
    arm_navigation_msgs::CollisionObject object;
    object = pm_->tidy_cfg_[*i];
      
    object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    collision_object_pub_.publish(object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();

    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
  }
  //==========  
  if(!for_motion_planning)
    return true;
  
  std::vector<std::string> name_parts_0;
  boost::split( name_parts_0, name_parts.at(0), boost::is_any_of("_") );// e.g from "GRASPED_CAN1" to "GRASPED" and "CAN1"

  std::string state_name = name_parts_0.at(0);
  std::string object_id = name_parts_0.at(1);

  if( !strcmp(state_name.c_str(), "Released") )// If source state of edge e is RELEASED then the released_objects _must_ be in tidyspot, this is for TRANSIT
  {
    // Set the released object in the tidy spot
    arm_navigation_msgs::CollisionObject object;
    object = pm_->tidy_cfg_[object_id];
      
    object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    collision_object_pub_.publish(object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();

    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
  }
  else if( !strcmp(state_name.c_str(), "Grasped") )// If the sources state of edge e is GRASPED, the object _must_be grasped.
  {
    // Update planning_environment
    arm_navigation_msgs::AttachedCollisionObject att_object;
    
    att_object.link_name = "link_rhand_palm";
    //att_object.touch_links.push_back("r_gripper_palm_link");
    
    att_object.object = pm_->messy_cfg_[object_id];// Assume that Grasped_XXX is always at messy_cfg_
//    att_object.object.header.frame_id = "link_rhand_palm";
    att_object.object.header.stamp = ros::Time::now();
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;    
//    // update (overwrite) the pose to conform with frame_id=link_rhand_palm
//    geometry_msgs::Pose pose;
//    pose.position.x = 0.0;
//    pose.position.y = 0.0;
//    pose.position.z = 0.0;
//    pose.orientation.x = 0;
//    pose.orientation.y = 0;
//    pose.orientation.z = 0;
//    pose.orientation.w = 1;
//    att_object.object.poses.push_back(pose);
        
    att_collision_object_pub_.publish(att_object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();
    
    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
  }
  
  ROS_DEBUG("set_planning_env: END");
  return true;
}

PlannerManager* pm_;
//! A publisher for collision objects
/*!
  A publisher should be initialized at the contructor (?)
*/
ros::Publisher collision_object_pub_;
//! A publisher for attached collision objects
/*!
  A publisher should be initialized at the contructor (?)
*/
ros::Publisher att_collision_object_pub_;
//! A publisher for visualizing a motion plan
/*!
  A publisher should be initialized at the contructor (?)
*/
ros::Publisher motion_plan_pub_;
//! A client that is used at several places.
/*!
  More...
*/
ros::ServiceClient set_planning_scene_diff_client_;
//! A helper variable
/*!
  Being used at several places.
*/
arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req_;

//! A helper variable
/*!
  Being used at several places.
*/
arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res_;
};// End of: GeometricPlannerManager class

#endif // #ifndef GEO_PLANNER_MANAGER_HPP_INCLUDED
