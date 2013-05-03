#ifndef GEO_PLANNER_MANAGER_HPP_INCLUDED
#define GEO_PLANNER_MANAGER_HPP_INCLUDED

#include <tf/transform_listener.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <boost/algorithm/string/erase.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include "planner_manager.hpp"
#include "tmm_utils.hpp"
#include "utils.hpp"
#include "hiro_utils.hpp"
#include "data.hpp"
#include "data_collector.hpp"

static const double COL_OBJ_PUB_TIME = 1.0;// prev. 1.0, 2.0
static const std::string SET_PLANNING_SCENE_DIFF_SRV_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_PLANNING_SCENE_SRV_NAME = "/environment_server/get_planning_scene";
static const std::string GET_ROBOT_STATE_SRV_NAME = "/environment_server/get_robot_state";
static const std::string TRAJECTORY_FILTER_SRV_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

static const size_t NUM_PLANNING_ATTEMPTS = 1;
static const double ALLOWED_PLANNING_TIME = 0.1 * 60.;
static const double ALLOWED_SMOOTHING_TIME = 2.0;
static const size_t MAX_JSPACE_DIM = 7;
static const double EST_HIGHEST_RESULT_COST = 5.0;
static const double MOTION_PLANNING_FAILURE_PENALTY = ALLOWED_SMOOTHING_TIME + EST_HIGHEST_RESULT_COST;

static boost::mt19937 gen( std::time(0) );

struct GeoPlanningCost
{
  GeoPlanningCost()
  :process(0.), result(0.)
  { }
  
  double process;
  double result;
  
  double
  total()
  {
    return (process + result);
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
  joint_states_cmd_pub_ = pm_->nh_.advertise<sensor_msgs::JointState>("joint_state_planning", 1);
  
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SRV_NAME);
  set_planning_scene_diff_client_ = pm_->nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_SRV_NAME);
}

bool
plan(TMMEdge e)
{
  // Check whether this edge is already geometrically planned, having a positive cost
  if(get(edge_weight,pm_->tmm_,e) > 0.)
  {
    ROS_DEBUG_STREAM("Geo. plan for e " << get(edge_name,pm_->tmm_,e) << "[" << get(edge_jspace,pm_->tmm_,e) << "]= already DONE.");
    return true;
  }
  
  ROS_DEBUG_STREAM("Geo. plan for e " << get(edge_name,pm_->tmm_,e) << "[" << get(edge_jspace,pm_->tmm_,e) << "]= BEGIN.");
  ros::Time planning_begin = ros::Time::now();
  
  // Get the start point /subseteq vertex_jstates (source of e) based on jspace of this edge e
  sensor_msgs::JointState start_state;
  start_state = get_jstate_subset(  get(edge_jspace, pm_->tmm_, e), get(vertex_jstates, pm_->tmm_, source(e, pm_->tmm_))  );
//  ROS_DEBUG_STREAM("start_state: SET with " << start_state.position.size() << " joints");
  
  // Determine all possible goal poses
  std::vector<sensor_msgs::JointState> goal_set;// Note that we do not use std::set because of a ros-msg constraint that is an array[] is handled as a vector
  
  std::vector<std::string> target_vertex_name_parts;
  boost::split( target_vertex_name_parts,get(vertex_name,pm_->tmm_,target(e,pm_->tmm_)),boost::is_any_of("_"),boost::token_compress_on );

  std::string main_state_str = target_vertex_name_parts.at(0);
  if( !strcmp(main_state_str.c_str(),"Grasped") or !strcmp(main_state_str.c_str(),"Released") )
  {
    // Call grasp/ungrasp planner here!
    goal_set = get_goal_set( source(e,pm_->tmm_),target(e,pm_->tmm_),get(edge_jspace,pm_->tmm_,e) ); 
  }
  else// must be either TidyHome or TmpHome_RBTID[xxx]
  {
    // Get a subset of initial jstates stored in the goal vertex, which is the home pose
    goal_set.push_back(   get_jstate_subset(  get(edge_jspace, pm_->tmm_, e), get(vertex_jstates, pm_->tmm_, target(e, pm_->tmm_))  )   );
  }
  ROS_DEBUG_STREAM("goal_set.size()= " << goal_set.size());
      
  if( goal_set.empty() )
  {
    ROS_INFO("goal_set is empty, no further motion planning, return!");
    ROS_DEBUG_STREAM("Geo. plan for e " << get(edge_name,pm_->tmm_,e) << "[" << get(edge_jspace,pm_->tmm_,e) << "]= END (false).");
    return false;
  }
  else
  { }
  
  // MOTION PLANNING ================================================================================================================================
  // Prepare the planning environment to do motion planning
  set_planning_env( source(e,pm_->tmm_) );
  
  // Put the geometric config of source vertex set above into edge prop.
  // Note that this is also an update because the edge_srcstate has been set when its source vertex's state is updated
  put(  edge_srcstate,pm_->tmm_,e,get_planning_env_str( source(e,pm_->tmm_) )  );
  
  // Obtain the best motion plan
  // Assume that the motion planner always return the best motion plan for every start and goal poses
  // Assume that the higher the manipulability the more likely the chance to obtain a better motion plan
  ROS_DEBUG("Start planning motions");
  
  // For the motion plan
  trajectory_msgs::JointTrajectory plan;
  GeoPlanningCost cost;
  bool found = false;
  
  // For calculating iter_cost
  size_t n_attempt = 0;
  
  // Try to plan a motion for all possible goal states in the goal set. Then, take the best among them.
  // This means that we consider a goal set not only a single goal state.
  for(std::vector<sensor_msgs::JointState>::const_iterator i=goal_set.begin(); i!=goal_set.end(); ++i)
  {
    ++n_attempt;
    ROS_INFO_STREAM( "Attempt: " << n_attempt << "-th of " << goal_set.size() );
  
    found = false;
    sensor_msgs::JointState goal_state = *i;
    
    if(  plan_motion( start_state,goal_state,get(edge_jspace,pm_->tmm_,e),&plan,&cost,&found )  )
    {
      if(found)
        break;// from iterating over the goal set; struggling exactly only for one motion plan solution
    }
    else
    {
      ROS_WARN("A call to /ompl_planning/plan_kinematic_path and or /benchmark_motion_plan: FAILED.");
    }
  }
  ROS_INFO_STREAM("n_attempt= " << n_attempt);
  
  if( !found ) // even after considering all goal poses in the goal_set
  {
    ROS_INFO_STREAM("No motion plan for " << goal_set.size() << " goals for e= " << get(edge_name,pm_->tmm_,e) << " in " << get(edge_jspace,pm_->tmm_,e));
    
    // Note that although there is no motion plan found, the cost is defined as (set at plan_motion())
    //cost->result = 0.;
    //cost->process = jspace_cost + planning_time + MOTION_PLANNING_FAILURE_PENALTY;
    
    put(edge_color,pm_->tmm_,e,std::string("red"));// geometrically validated but no motion plan
  }
  else
  {
    put(edge_color,pm_->tmm_,e,std::string("green"));// geometrically validated and there exists, at least, one motion plan
  }
  
  // Calculate the cost of iterating over the goal set
  // Because we only strive for 1 success, n_failure is always n_attempt-1
  double iter_cost;
  iter_cost = exp( (double)(n_attempt-1)/(double)(n_attempt) );
  
  // Sanity check isNan
  if( (cost.total()+iter_cost) != (cost.total()+iter_cost) )
  {
    ROS_ERROR_STREAM("iter_cost= " << iter_cost);
    ROS_ERROR_STREAM("cost.process= " << cost.process);
    ROS_ERROR_STREAM("cost.result= " << cost.result);
    ROS_ERROR("nan detected in edge cost");
    return false;
  }
  
  // Reset the planning environment
  reset_planning_env();
  
  // Sum up + Put the best motion plan of this edge e and its geo. planning cost
  double planning_time;// refers to motion_planning + grasp planning time
  planning_time = (ros::Time::now()-planning_begin).toSec();  

  put( edge_plan,pm_->tmm_,e,plan );
  put( edge_planstr,pm_->tmm_,e,get_planstr(plan) );
  put( edge_weight,pm_->tmm_,e,cost.total()+iter_cost );
  put( edge_mptime,pm_->tmm_,e,planning_time );

  ROS_DEBUG_STREAM("Geo. plan for e " << get(edge_name,pm_->tmm_,e) << "[" << get(edge_jspace,pm_->tmm_,e) << "]= END (true).");  
  return true;
}
//! Obtain samples as the search progresses, get samples from paths from (root,root+1, ..., v) to adjacent of v
/*!
  Specifically,
  samples are collected from all unique paths that are subset of the path from the root the the vertex v and have not yet used as a sample before.
  
  Concretely,
  consire the path: 
  ROOT --> A 
  then only one path: ROOT --> A 
  ROOT --> A --> B
  then there are 2 paths that can form new samples: ROOT--> A --> B and A --> B
  
  Therefore, the gist is that we have to find the path from the root to the vertex v
*/
bool
get_samples_online(TMMVertex v,Data* samples)
{
//  ROS_DEBUG("get_samples_online(): BEGIN");
  
  TaskMotionMultigraph tmm;
  tmm = pm_->tmm_;
  
  // Remove more-expensive edges, remove parallelism
  // Because the solution path should always choose the cheapest,
  // or does the dfs do it already?
  std::set<TMMEdge> tobe_removed_edges;
  
  boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
  for(boost::tie(vi,vi_end) = vertices(tmm); vi!=vi_end; ++vi)
  {
    std::map<TMMVertex,TMMEdge> tv_e_map;
    
    graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
    for(tie(oei,oei_end) = out_edges(*vi,tmm); oei!=oei_end; ++oei )
    {
      std::map<TMMVertex,TMMEdge>::iterator it;
      bool inserted;
      
      tie(it,inserted) = tv_e_map.insert( std::make_pair(target(*oei,tmm),*oei) );
      
      if(!inserted)
      {
        if(get(edge_weight,tmm,*oei) < get(edge_weight,tmm,it->second))
        {
          tobe_removed_edges.insert(it->second);

          it->second = *oei;
        }
        else
        {
          tobe_removed_edges.insert(*oei);
        }
      }
    }
  }// end of: For each vertex in tmm
  
  for(std::set<TMMEdge>::const_iterator i=tobe_removed_edges.begin(); i!=tobe_removed_edges.end(); ++i)
    remove_edge(*i,tmm);
//  ROS_DEBUG("Parallelism: removed");
  
//  // Filter only the planned edge to make dfs_visit more efficient by cutting the depth of the tmm
//  PlannedEdgeFilter<TMMEdgeColorMap> planned_edge_filter( get(edge_color, tmm) );
//  typedef filtered_graph< TaskMotionMultigraph, PlannedEdgeFilter<TMMEdgeColorMap> > PlannedTMM;

//  PlannedTMM p_tmm(tmm, planned_edge_filter);
//  ROS_DEBUG_STREAM("num_vertices(p_tmm)= " << num_vertices(p_tmm));
//  ROS_DEBUG_STREAM("num_edges(p_tmm)= " << num_edges(p_tmm));

// Extract training samples from hot_paths: sequence of edges from tmm_root_ to the adjacent vertices of v, which are the targets of the just planned edges
  graph_traits<TaskMotionMultigraph>::adjacency_iterator avi, avi_end;
  for(tie(avi,avi_end)=adjacent_vertices(v,tmm); avi!=avi_end; ++avi )  
  {
    boost::graph_traits<TaskMotionMultigraph>::vertex_descriptor goal;
    goal = *avi;
    
    std::vector<TMMEdge> hot_path;
    
    DFSVisitor<TaskMotionMultigraph> vis(&goal,&hot_path);
    
    try
    {
      depth_first_search( tmm,visitor(vis) );
    }
    catch(DFSFoundGoalSignal fg)
    { }
    
//    ROS_DEBUG("hot_path= ");
//    for(std::vector<TMMEdge>::const_iterator i=hot_path.begin(); i!=hot_path.end(); ++i)
//      ROS_DEBUG_STREAM("e(" << get(vertex_name,tmm,source(*i,tmm)) << "," << get(vertex_name,tmm,target(*i,tmm)) << "), ");

    std::string metadata_path;
    metadata_path = "/home/vektor/rss-2013/data/ref/metadata.csv";
    
    DataCollector<TaskMotionMultigraph> dc;
    dc.get_samples(hot_path,tmm,metadata_path,samples);
    
  }// end of: for_each avi
    
//  ROS_DEBUG("get_samples_online(): END");
  return true;
}

//! Obtain features
/*!
  As the heuristic is from the learning machine.
  We have to give the learning machine features.
  
  One way to predict the optimal solution path is by traversing in dfs manner.
  Annother is to use random (uniformly) whenever there is a tie that should be broke.
  
  Put this in geo_planner_manager.cpp because it can access pm_->tmm_.
  Although, this is not really appropriate.
*/
bool
get_fval(TMMVertex v,Input* in)
{
  ROS_DEBUG_STREAM("get_fval() with head= " << get(vertex_name,pm_->tmm_,v));  
  
  // Predict the optimal solution path from this vertex v
  std::vector<TMMEdge> est_opt_sol_path;
  predict_opt_sol_path(v,&est_opt_sol_path);

//  ROS_DEBUG("predict_opt_sol_path= ");
//  for(std::vector<TMMEdge>::const_iterator i=est_opt_sol_path.begin(); i!=est_opt_sol_path.end(); ++i)
//    ROS_DEBUG_STREAM("e(" << get(vertex_name,pm_->tmm_,source(*i,pm_->tmm_)) << "," << get(vertex_name,pm_->tmm_,target(*i,pm_->tmm_)) << "), ");
  
  if( est_opt_sol_path.empty() )
  {
    ROS_WARN("est_opt_sol_path.empty()= TRUE");
    return false;
  }
  
    
  // Extract feature values from the estimated optimal-solution path, which is as the heuristic path
  std::string metadata_path;
  metadata_path = "/home/vektor/rss-2013/data/ref/metadata.csv";

  DataCollector<TaskMotionMultigraph> dc;
  bool success;
  success = dc.get_fval(est_opt_sol_path,pm_->tmm_,metadata_path,in);
  
  return success;
}

void
predict_opt_sol_path(const TMMVertex& v,std::vector<TMMEdge>* est_opt_sol_path)
{
  if(v == pm_->tmm_goal_)
    return;
  
  std::vector<TMMVertex> adj_vertices;
  
  graph_traits<TaskMotionMultigraph>::adjacency_iterator avi, avi_end;
  for(tie(avi,avi_end)=adjacent_vertices(v,pm_->tmm_); avi!=avi_end; ++avi )
  {
    adj_vertices.push_back(*avi);
  }
  
  if(adj_vertices.empty())
    return;
  
  // Randomly choose one adjacent vertice uniformly
  boost::uniform_int<> dist(0,adj_vertices.size()-1);
  boost::variate_generator< boost::mt19937&, boost::uniform_int<> > rnd(gen,dist);
  
  TMMVertex chosen_av;
  chosen_av = adj_vertices.at( rnd() );
  
  // TODO should Bias to choose the smallest jspace
  // Note that tmm_ is a multigraph, which edge does edge() return?
  TMMEdge e;
  bool found;// useless because it must exist
  boost::tie(e,found) = edge(v,chosen_av,pm_->tmm_);
  
  est_opt_sol_path->push_back(e);
  
  // Recursive call till the goal
  predict_opt_sol_path(chosen_av,est_opt_sol_path);
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
  The given vertex is the just-examined vertex.
  Update jstates of adjacent vertex av of this vertex v to the cheapest just-planned edge, note copy jstates of v to adjacent vertices first!'
  
  
  PLUS,
  put the state at the adjacent vertices to the srcstate property of out edges
*/
bool
set_av_jstates(TMMVertex v)
{
  ROS_DEBUG_STREAM("Updating adjacent_vertices of v  " << get(vertex_name,pm_->tmm_,v) << ": Begin.");
  
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
//  ROS_DEBUG_STREAM("av_ce_map.size()= " << av_ce_map.size());
//  for(std::map<TMMVertex,TMMEdge>::iterator i=av_ce_map.begin(); i!=av_ce_map.end(); ++i)
//    ROS_DEBUG_STREAM(get(vertex_name,pm_->tmm_,i->first) << " <---> " << get(edge_jspace,pm_->tmm_,i->second));
    
  // Update the adjacent vertices's jstates using av_ce_map
  for(std::map<TMMVertex,TMMEdge>::iterator i=av_ce_map.begin(); i!=av_ce_map.end(); ++i)  
  {
    ROS_DEBUG_STREAM("Cheapest edge of av " << get(vertex_name,pm_->tmm_,i->first) << "= " << get(edge_jspace,pm_->tmm_,i->second));

    // Copy all joint states first, in the case only a subset of jstate that is being updated
    put( vertex_jstates,pm_->tmm_,i->first,get(vertex_jstates,pm_->tmm_,v) );
    
    sensor_msgs::JointState joint_state;
    joint_state = get(vertex_jstates,pm_->tmm_,i->first);
    
    // The updating_joint_state is at the last point (=goal point) of the motion planning (path/trajectory) of the action of an edge
    trajectory_msgs::JointTrajectory motion_plan;
    motion_plan = get(edge_plan,pm_->tmm_,i->second);
    
    if( !motion_plan.points.empty() )
    {
      // Update vertex jstate
      trajectory_msgs::JointTrajectoryPoint goal_point = motion_plan.points.back();
      
      for(std::vector<std::string>::const_iterator ii=motion_plan.joint_names.begin(); ii!=motion_plan.joint_names.end(); ++ii)
      {
        std::vector<std::string>::iterator j;
        j = std::find(joint_state.name.begin(), joint_state.name.end(), *ii);
        
        joint_state.position.at( j-joint_state.name.begin() ) = goal_point.positions.at( ii-motion_plan.joint_names.begin() );
      }
      
      put(vertex_jstates,pm_->tmm_,i->first,joint_state);
    }
    
    // Put this vertex state in the srcstate property of out edges
    // Note that if there is no motion plan than there is no update meaning that jstate in the start and goal states are same
    graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
    for(tie(oei,oei_end) = out_edges(i->first,pm_->tmm_); oei!=oei_end; ++oei)
    {
      put(  edge_srcstate,pm_->tmm_,*oei,get_planning_env_str(i->first)  );
    }
  }// end of: each avi in av_ce_map
  
  ROS_DEBUG_STREAM("Updating adjacent_vertices of v " << get(vertex_name,pm_->tmm_,v) << ": END");
  return true;
}

void
mark_vertex(const TMMVertex& v)
{
  put(vertex_color, pm_->tmm_, v, color_traits<boost::default_color_type>::black());// for examined vertex
}

//! Obtain the description of the planning environment in the form of string
/*!
  There are 2 descriptors of the planning environment, namely:
  (1) wstate: workspace state, poses of manipulated objects
  (2) jstate: joint states
  
  This is for extracting features.
*/
std::string
get_planning_env_str(const TMMVertex& v)
{
  ROS_DEBUG_STREAM("get_planning_env_str(" << get(vertex_name,pm_->tmm_,v) << "): BEGIN");
  
  std::string state_str;
  
  // We assume that there is no uncertainty
  // Therefore, object poses are just based on vertex's wstate prop. set at the beginning 
  std::vector<arm_navigation_msgs::CollisionObject> wstate;
  wstate = get( vertex_wstate,pm_->tmm_,v );
  
  for(std::vector<arm_navigation_msgs::CollisionObject>::const_iterator i=wstate.begin(); i!=wstate.end(); ++i)
  {
    state_str += i->id + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).position.x) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).position.y) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).position.z) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).orientation.x) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).orientation.y) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).orientation.z) + ",";
    state_str += boost::lexical_cast<std::string>(i->poses.at(0).orientation.w) + ";";// The last one use ";", instead of ","
  }
  
  // Put joint states
  sensor_msgs::JointState joint_state;  
  joint_state = get( vertex_jstates,pm_->tmm_,v );
  
  for(std::vector<std::string>::const_iterator i=joint_state.name.begin(); i!=joint_state.name.end(); ++i)
  {
    // Sanity check isNan
    if( (joint_state.position.at(i-joint_state.name.begin())) != (joint_state.position.at(i-joint_state.name.begin())) )
    {
      ROS_ERROR_STREAM("nan detected in joint= " << *i);
      state_str = "NAN DETECTED";
      return state_str;
    }
    
    state_str += *i + ",";
    state_str += boost::lexical_cast<std::string>( joint_state.position.at(i-joint_state.name.begin()) ) + ";";
  }
  boost::erase_last(state_str,";");
  
  ROS_DEBUG_STREAM("get_planning_env_str(): END");  
  return state_str;
} 
//!
/*!
  Mainly, this initialize the state in a vertex.
  The state is represented in 2 entities: (1) jstates and (2) wstate
  (1) jstates contains joint states of the robot, this is initialized with jstates(MessyHome), later they will be updated
  (2) wstate is set as it is, no further update is required since we assume no action-uncertainty here!
*/
void
init_vertex(const TMMVertex& v)
{
//  ROS_DEBUG_STREAM("init_vertex(" << get(vertex_name,pm_->tmm_,v) << "): BEGIN");
  
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
  
  // Sanity check
  for(size_t i=0; i<res.robot_state.joint_state.name.size(); ++i)
  {
    if( res.robot_state.joint_state.position.at(i) != res.robot_state.joint_state.position.at(i) )
    {
      ROS_ERROR_STREAM("pos= " << res.robot_state.joint_state.position.at(i));
      ROS_ERROR_STREAM("name= " << res.robot_state.joint_state.name.at(i));
      ROS_ERROR("nan detected when initializing vertices, return!");
      return;
    }
  }

  put(vertex_jstates, pm_->tmm_, v, res.robot_state.joint_state);
  
  //Set wstate based on the vertex's name
  std::string name = get(vertex_name, pm_->tmm_, v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );// e.g from "GRASPED_CAN1[CAN2.CAN3.]" to "GRASPED_CAN1" and "CAN2.CAN3.]"

  std::vector<std::string> tidied_object_ids;  
  if(name_parts.size()==2)
  {
    boost::split( tidied_object_ids, name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to CAN2 and CAN3 and ]
    tidied_object_ids.erase(tidied_object_ids.end()-1);//remove a "]"
  }
  
  std::vector<arm_navigation_msgs::CollisionObject> wstate;
  
  for(std::map<std::string,arm_navigation_msgs::CollisionObject>::const_iterator i=pm_->movable_obj_messy_cfg_.begin(); i!=pm_->movable_obj_messy_cfg_.end(); ++i)
  {
    std::vector<std::string>::iterator it;
    it = std::find(tidied_object_ids.begin(),tidied_object_ids.end(),i->first);
    
    if( it==tidied_object_ids.end() )
      wstate.push_back(i->second);
    else
      wstate.push_back( pm_->movable_obj_tidy_cfg_[i->first] );
  }
  
  for(std::map<std::string,arm_navigation_msgs::CollisionObject>::const_iterator i=pm_->unmovable_obj_cfg_.begin(); i!=pm_->unmovable_obj_cfg_.end(); ++i)
  {
      wstate.push_back( pm_->unmovable_obj_cfg_[i->first] );
  }
  
  // Plus setting wstate with unmovable object listed in messy.cfg file 
//  std::string data_path;
//  if( !ros::param::get("/data_path", data_path) )
//    ROS_WARN("Can not get /data_path, use the default value instead");
//  
//  utils::ObjCfg all_obj_cfg;
//  utils::read_obj_cfg(std::string(data_path+"/messy.cfg"),&all_obj_cfg);
//  
//  utils::ObjCfg unmovable_obj_cfg;
//  for(utils::ObjCfg::const_iterator i=all_obj_cfg.begin(); i!=all_obj_cfg.end(); ++i)
//  {
//    std::string id;
//    id = i->id;
//    
//    b
//  }
  
  
//  for(std::vector<arm_navigation_msgs::CollisionObject>::const_iterator i=wstate.begin(); i!=wstate.end(); ++i)
//  {
//    ROS_DEBUG_STREAM("id= " << i->id);
//    ROS_DEBUG_STREAM("i->poses.at(0).position.x= " << i->poses.at(0).position.x);
//    ROS_DEBUG_STREAM("i->poses.at(0).position.y= " << i->poses.at(0).position.y);
//    ROS_DEBUG_STREAM("i->poses.at(0).position.z= " << i->poses.at(0).position.z);
//    ROS_DEBUG_STREAM("i->poses.at(0).orientation.x= " << i->poses.at(0).orientation.x);
//    ROS_DEBUG_STREAM("i->poses.at(0).orientation.y= " << i->poses.at(0).orientation.y);
//    ROS_DEBUG_STREAM("i->poses.at(0).orientation.z= " << i->poses.at(0).orientation.z);
//    ROS_DEBUG_STREAM("i->poses.at(0).orientation.w= " << i->poses.at(0).orientation.w);
//  }
  
  put(vertex_wstate,pm_->tmm_,v,wstate);
}

void
put_heu(TMMVertex v, const double& h)
{
  put(vertex_heu,pm_->tmm_,v,h);
}

void
remove_ungraspable_edge(TMMEdge e)
{
  ROS_DEBUG_STREAM("Remove e " << get(edge_name,pm_->tmm_,e) << "[" << get(edge_jspace,pm_->tmm_,e) << "]");
  remove_edge(e,pm_->tmm_);
}

private:
//! Obtain jstate subset from the whole joint-state of the robot 
/*!
  Note that this dual-arm robot entirely has 6+6+1+2=15 joints.
  Therefore, to activate one arm at a time, we have to obtain a subset for the activated arm from the entire joint space.
  
  In addition, in one activated arm, there are 2 possible joint space: "rarm_U_chest" and "rarm".
*/
sensor_msgs::JointState
get_jstate_subset(const std::string& jspace, const sensor_msgs::JointState& jstates)
{
  sensor_msgs::JointState subset_jstates;
  
  // Interpret jspace names
  // TODO use get_joints_in_group (planning_environment_msgs/GetJointsInGroup) srv OR /robot_description_planning/groups parameters
  // TODO http://answers.ros.org/question/12522/planning_environment_msgs-in-electric/
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
  else if( !strcmp(jspace.c_str(), "larm_U_chest") )
  {
    subset_jstates.name.push_back("joint_chest_yaw");
    
    subset_jstates.name.push_back("joint_lshoulder_yaw");
    subset_jstates.name.push_back("joint_lshoulder_pitch");
    subset_jstates.name.push_back("joint_lelbow_pitch");
    subset_jstates.name.push_back("joint_lwrist_yaw");
    subset_jstates.name.push_back("joint_lwrist_pitch");
    subset_jstates.name.push_back("joint_lwrist_roll");
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
  else if( !strcmp(jspace.c_str(), "larm") )
  {
    subset_jstates.name.push_back("joint_lshoulder_yaw");
    subset_jstates.name.push_back("joint_lshoulder_pitch");
    subset_jstates.name.push_back("joint_lelbow_pitch");
    subset_jstates.name.push_back("joint_lwrist_yaw");
    subset_jstates.name.push_back("joint_lwrist_pitch");
    subset_jstates.name.push_back("joint_lwrist_roll");
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

//  for(std::vector<std::string>::iterator i=subset_jstates.name.begin(); i!=subset_jstates.name.end(); ++i)
//  {
//    std::vector<std::string>::iterator j;
//    j = std::find(jstates.name.begin(),jstates.name.end(),i->c_str());

//    subset_jstates.position.at( i-subset_jstates.name.begin() ) = jstates.position.at( j-jstates.name.begin() );
//  }
  
  return subset_jstates;
}

//! Obtain a goal set cointaingin goal(grasp/ungrasp) poses
/*!
  \param &sv source vertex, for setting the planning environment, in particulat the robot_state
  \param &v
  \param &jspace
  \return a vector of goal(grasp/ungrasp) poses in sensor_msgs::JointState
  
*/
std::vector<sensor_msgs::JointState>
get_goal_set(const TMMVertex& sv,const TMMVertex& v, const std::string& jspace)
{
  std::vector<sensor_msgs::JointState> goal_set;// Because plan_grasp receive a vector, instead of a set
  
  // Set the planning environment (for grasp planning) so as to satisfy the init_state, i.e. by setting already-tidied-up object at tidy spot.
  set_planning_env(sv,v);
  
  // Call grasp planner or ungrasp planner depending on out_the name of v that is the state
  std::string name = get(vertex_name, pm_->tmm_, v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );// e.g from "Grasped_CAN1_RBT[CAN2.CAN3.]" to "Grasped_CAN1_RBT" and "CAN2.CAN3.]"
  
  if( !strcmp(name_parts.at(0).c_str(),"TidyHome") )
  { }
  else
  {
    std::vector<std::string> name_parts_0;
    boost::split( name_parts_0, name_parts.at(0), boost::is_any_of("_") );// e.g from "Grasped_CAN1_RBT" to "Grasped", "CAN1" and "RBT"

    std::string state_name = name_parts_0.at(0);
    std::string object_id = name_parts_0.at(1);
    std::string rbt_id = name_parts_0.at(2);
    
    if( !strcmp(state_name.c_str(), "Grasped") )// Assume that GRASP is always in messy_spot
    {
      arm_navigation_msgs::CollisionObject object;
      object = pm_->movable_obj_messy_cfg_[object_id];
    
      plan_grasp(object,rbt_id,jspace,&goal_set);
    }
    else if( !strcmp(state_name.c_str(), "Released") )// Assume that UNGRASP is always in tidy_spot
    {
      arm_navigation_msgs::CollisionObject object;
      object = pm_->movable_obj_tidy_cfg_[object_id];
      
      plan_ungrasp(object,rbt_id,jspace,&goal_set);
    }
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
plan_grasp(arm_navigation_msgs::CollisionObject& object, const std::string& rbt_id, const std::string& jspace, std::vector<sensor_msgs::JointState>* grasp_poses)
{
  // Call the plan_grasp srv
  grasp_planner::PlanGrasp::Request req;
  grasp_planner::PlanGrasp::Response res; 
  
  req.object = object;
  req.rbt_id = rbt_id;
  req.jspace = jspace;
  
  ros::service::waitForService("/plan_grasp");
  ros::ServiceClient client;
  client = pm_->nh_.serviceClient<grasp_planner::PlanGrasp>("/plan_grasp");
    
  if ( client.call(req,res) )
  {
    ROS_DEBUG("Succeeded to call plan_grasp service");
    
    if( res.grasp_plans.empty() )
      ROS_WARN("Although done successfully, but _no_ grasp plan.");
    
    *grasp_poses = res.grasp_plans;// This may be empty
    
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
plan_ungrasp(arm_navigation_msgs::CollisionObject& object, const std::string& rbt_id, const std::string& jspace, std::vector<sensor_msgs::JointState>* ungrasp_poses)
{
  // Set the object in the tidy_cfg in the planning_environment
  object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
  collision_object_pub_.publish(object);
  ros::Duration(COL_OBJ_PUB_TIME).sleep();

  if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    
  return plan_grasp(object,rbt_id,jspace,ungrasp_poses); 
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
  
//  // DEBUG
//  cerr << "jspace= " << jspace << endl;
//  cerr << "goal_state inside plan_motion()" << endl;
//  utils::print_robot_state(goal_state);
  
  ros::Time planning_begin = ros::Time::now();
  if ( planning_client.call(req, res) )
  {
    double planning_time;
    planning_time = (ros::Time::now()-planning_begin).toSec();

    // Calculate jspace_cost
    size_t jspace_dim;
    jspace_dim = start_state.name.size(); // _must_ be equal to goal_state.name.size()
    
    double jspace_cost;
    jspace_cost = exp( (double)jspace_dim/(double)MAX_JSPACE_DIM );
          
    if ( res.trajectory.joint_trajectory.points.empty() )
    {
      ROS_INFO_STREAM("Motion planner was unable to plan a path to goal, joint_trajectory.points.size()= " << res.trajectory.joint_trajectory.points.size());
      *found = false;
      
      // Accumulate the cost 
      cost->result = 0.;
      cost->process = jspace_cost + planning_time + MOTION_PLANNING_FAILURE_PENALTY;
    }
    else
    {
      ROS_INFO("Motion planning succeeded");
      *found = true;
      
//      utils::print_robot_state(res.trajectory.joint_trajectory,res.trajectory.joint_trajectory.points.size()-1);
      
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
      cost->result = benchmark_path_res.length;// + benchmark_path_res.smoothness + benchmark_path_res.clearance
      cost->process = jspace_cost + (planning_time + smoothing_time);// in the text, smoothing time is included within planning_time
      
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
    ROS_ERROR("Motion planning service call failed on %s",planning_client.getService().c_str());
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
  Preferable order:
  (1) Reset the workspace
  (2) Reset the robot state 
  
  Although it may in any order, resetting the workspace first seems to produce better-looking planning process.
  
  \return whether successful
*/
bool
reset_planning_env()
{
  // Reset the workspace===================================================================================================================
  for(std::map<std::string, arm_navigation_msgs::CollisionObject>::iterator i=pm_->movable_obj_messy_cfg_.begin(); i!=pm_->movable_obj_messy_cfg_.end(); ++i)
  {
    i->second.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    
    collision_object_pub_.publish( i->second );
    ros::Duration(COL_OBJ_PUB_TIME).sleep();
    
    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    {
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
      return false;
    }
    
    // Note that this brutal reset leads to: e.g.
    //[WARN] No attached body CAN1 attached to link link_rhand_palm
    //Sanity check failing - no entry in acm for collision space object CAN1
    //No entry in default collision matrix for attached body CAN1 when there really should be.
    //Must already have an entry in allowed collision matrix for CAN1
    
    arm_navigation_msgs::AttachedCollisionObject att_object;
    
    // For the lhand
    att_object.link_name = "link_lhand_palm";
    //att_object.touch_links.push_back("l_gripper_palm_link");
    att_object.object.id = i->first;
    att_object.object.header.frame_id = "link_lhand_palm";
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
    
    // For the rhand
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
  
  // Reset the robot state===================================================================================================================
  if(!set_robot_state(pm_->tmm_root_))
    return false;
  
  return true;
}

bool
set_workspace(const TMMVertex& v,const bool sim_grasped_or_released=false)
{
  // This part only sets tidied objects (everything inside ...[]) to tidied positions
  // Only set the workspace; excluding the robot
  std::string name = get(vertex_name, pm_->tmm_, v);
  
  std::vector<std::string> name_parts;
  boost::split( name_parts, name, boost::is_any_of("[") );// e.g from "GRASPED_CAN1[CAN2.CAN3.]" to "GRASPED_CAN1" and "CAN2.CAN3.]"
  
  if( !strcmp(name_parts.at(0).c_str(),"MessyHome") )
    return true;
    
  std::vector<std::string> tidied_object_ids;
  boost::split( tidied_object_ids, name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to CAN2 and CAN3 and ]
  tidied_object_ids.erase(tidied_object_ids.end()-1);//remove a "]"

  for(std::vector<std::string>::const_iterator i=tidied_object_ids.begin(); i!=tidied_object_ids.end(); ++i)
  {
    arm_navigation_msgs::CollisionObject object;
    object = pm_->movable_obj_tidy_cfg_[*i];
      
    object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    collision_object_pub_.publish(object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();

    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    {
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
      return false;
    }
    
  }
  
  if( !sim_grasped_or_released or !strcmp(name_parts.at(0).substr(0,7).c_str(),"TmpHome") )// Note the actual name is TmpHome_RBT
    return true;
    
 // Set the workspace for simulate grasping or released=======================================================================================
  std::vector<std::string> name_parts_0;
  boost::split( name_parts_0, name_parts.at(0), boost::is_any_of("_") );// e.g from "Grasped_CAN1_RARM" to "GRASPED" and "CAN1" and "RARM"

  std::string state_name = name_parts_0.at(0);
  std::string object_id = name_parts_0.at(1);
  std::string rbt_id =name_parts_0.at(2);

  if( !strcmp(state_name.c_str(), "Released") )// If source state of edge e is RELEASED then the released_objects _must_ be in tidyspot, this is for TRANSIT
  {
    // Set the released object in the tidy spot
    arm_navigation_msgs::CollisionObject object;
    object = pm_->movable_obj_tidy_cfg_[object_id];
      
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
    
    att_object.link_name = get_eof_link(rbt_id);
    
//    // Method 1, WARN: this method is not stable i.e. a grasped object may at a strange pose
//    att_object.object.id = object_id;
//    att_object.object.header.frame_id = get_eof_link(rbt_id);
//    // update (overwrite) the pose to conform with frame_id=link_XXXX_palm
//    geometry_msgs::Pose pose;
//    pose.position.x = 0.0;
//    pose.position.y = 0.070;// grasp_padding = 0.070
//    pose.position.z = 0.0;
//    pose.orientation.x = 0;
//    pose.orientation.y = 0;
//    pose.orientation.z = 0;
//    pose.orientation.w = 1;
//    att_object.object.poses.push_back(pose);
//    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;

    // Method 2 [http://www.ros.org/wiki/arm_navigation/Tutorials/Planning%20Scene/Attaching%20Virtual%20Objects%20to%20the%20Robot]
    att_object.object = pm_->movable_obj_messy_cfg_[object_id];// Assume that Grasped_XXX is always at movable_obj_messy_cfg_
    att_object.object.header.frame_id = get_eof_link(rbt_id);
    // update (overwrite) the pose to conform with frame_id=link_XXXX_palm
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.070;// grasp_padding = 0.070
    pose.position.z = 0.0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    att_object.object.poses.at(0) = pose;
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.stamp = ros::Time::now();
      
    att_collision_object_pub_.publish(att_object);
    ros::Duration(COL_OBJ_PUB_TIME).sleep();
    
    if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
      ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    else
      ROS_DEBUG_STREAM("Grasped: " << object_id << ": " << rbt_id << "[" << get_eof_link(rbt_id) << "]");
  } 
  
  return true;
}

bool
set_robot_state(const TMMVertex& v)
{
//  // Set the robot joint states with the source vertex
//  joint_states_cmd_pub_.publish( get(vertex_jstates,pm_->tmm_,sv) );
//  ROS_DEBUG("set_planning_env: robot_state published");

  std::string data_path;
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
        
//  ROS_DEBUG_STREAM("Try to publish vertex_jstates. On " << data_path);
  std::cerr << "Publishing vertex_jstates .";
  bool passed = false;
  do
  {
    joint_states_cmd_pub_.publish( get(vertex_jstates,pm_->tmm_,v) );
    passed = true;
//    ROS_DEBUG_STREAM("jstates: published (trial).On " << data_path);
//    ros::Duration(0.2).sleep();
    
    arm_navigation_msgs::GetRobotState::Request req;
    arm_navigation_msgs::GetRobotState::Response res;

    ros::service::waitForService("/environment_server/get_robot_state");
    ros::ServiceClient get_rbt_state_client = pm_->nh_.serviceClient<arm_navigation_msgs::GetRobotState>("/environment_server/get_robot_state");
    
//    ROS_DEBUG("get_rbt_state_client.call(): BEGIN");
    if( !get_rbt_state_client.call(req, res) )
    { 
      ROS_WARN("Call to /environment_server/get_robot_state: FAILED");
    }
    else
    {
      for(std::vector<std::string>::const_iterator ii=res.robot_state.joint_state.name.begin(); ii!=res.robot_state.joint_state.name.end(); ++ii)
      {
        sensor_msgs::JointState jstates;
        jstates = get(vertex_jstates,pm_->tmm_,v);
      
        std::vector<std::string>::iterator j;
        j = std::find(jstates.name.begin(),jstates.name.end(),*ii);
        
//        ROS_DEBUG_STREAM("of X= " << jstates.position.at(j-jstates.name.begin()));
//        ROS_DEBUG_STREAM("of Y= " << res.robot_state.joint_state.position.at(ii-res.robot_state.joint_state.name.begin()));
        
        if( jstates.position.at(j-jstates.name.begin()) != res.robot_state.joint_state.position.at(ii-res.robot_state.joint_state.name.begin()) )
        {
//          ROS_DEBUG("joint_states_cmd_pub_.publish(): FAILED");
          std::cerr << ".";
          
          passed = false;
          break;
        }
      }
    }
  }
  while(!passed);
  std::cerr << std::endl;
  ROS_DEBUG("set_planning_env: robot_state published");
  
  return true;
}

//! For grasp/ungrasp planning
bool
set_planning_env(const TMMVertex& sv,const TMMVertex& tv)
{
  ROS_DEBUG("set_planning_env for GP: BEGIN");
  
  if( !set_workspace(tv) )
    return false;
    
  if(!set_robot_state(sv))
    return false;
  
  ROS_DEBUG("set_planning_env for GP: END");
  return true;
}

//! For motion planning
bool
set_planning_env(const TMMVertex& v)
{
  ROS_DEBUG("set_planning_env for MP: BEGIN");

  if( !set_robot_state(v) ) 
    return false;
  ROS_DEBUG("set_robot_state(): OK");
  
  bool sim_grasped_or_released = true;
  if ( !set_workspace(v,sim_grasped_or_released) )
    return false;
  ROS_DEBUG("set_workspace(): OK");
    
  ROS_DEBUG("set_planning_env for MP: END");
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

ros::Publisher joint_states_cmd_pub_;
};// End of: GeometricPlannerManager class

#endif // #ifndef GEO_PLANNER_MANAGER_HPP_INCLUDED
