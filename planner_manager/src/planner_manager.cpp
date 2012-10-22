//TODO change how messy_config_ and tidy_config_ are set!
#include <ros/ros.h>

#include <arm_navigation_msgs/CollisionObject.h>

#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <algorithm>
#include <functional>
#include <limits>

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/config.hpp>

#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
#error adjacency_list_io.hpp has not been ported to work with VC++
#endif

#include <boost/graph/adjacency_list_io.hpp>

#include <boost/graph/property_iter_range.hpp>

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include "hiro_sensor/Sense.h"
#include "task_planner/PlanTask.h"
#include "grasp_planner/PlanGrasp.h"
#include "hiro_control/ControlHand.h"
#include "hiro_control/MoveArm.h"
#include "hiro_common/BenchmarkPath.h"

using namespace boost;
using namespace std;

enum edge_flag_t { edge_flag };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, flag);
}

enum edge_solnums_t { edge_solnums };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, solnums);
}

//TODO is it possible to make a variable with more than one possible type. Yes, generic programming, but how exactly to apply to this case
struct HLAImplementation
{
  arm_navigation_msgs::RobotTrajectory motion_path;
  std::vector<sensor_msgs::JointState> grasp_poses;
};

enum edge_impl_t { edge_impl };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, impl);
}

enum {NOT_YET=0, PLANNED, PLANNED_BUT_FAILURE, BEST_SOLUTION};// for edge_flags
enum {MessyHome=0, TidyHome};// sync with the task_planner.cpp

typedef boost::property< edge_weight_t, double, property<edge_flag_t, size_t, property<edge_name_t, std::string> > > RawEdgeProperty;
typedef boost::property< edge_weight_t, double, property<edge_flag_t, size_t, property<edge_name_t, std::string, property<edge_impl_t, HLAImplementation, property<edge_color_t, default_color_type, property< edge_solnums_t, std::vector<size_t> > > > > > > EdgeProperty;

typedef boost::property<vertex_name_t, std::string> RawVertexProperty;
typedef boost::property<vertex_name_t, std::string, property<vertex_color_t, default_color_type> > VertexProperty; // Conflict with read() from adjacency_list_io.hpp

//TODO Consider alternatives; note that using bidirectionalS costs us the space twice of using directedS, is it worth it ?
typedef boost::adjacency_list<listS, vecS, bidirectionalS, RawVertexProperty, RawEdgeProperty > RawGraph;  
typedef boost::adjacency_list<listS, vecS, bidirectionalS, VertexProperty, EdgeProperty > Graph;  

typedef Graph::vertex_descriptor Vertex;
typedef Graph::edge_descriptor Edge;

typedef property_map<Graph, edge_name_t>::type EdgeNameMap;
typedef property_map<Graph, edge_weight_t>::type EdgeWeightMap;
typedef property_map<Graph, edge_flag_t>::type EdgeFlagMap;
typedef property_map<Graph, edge_impl_t>::type EdgeImplMap;
typedef property_map<Graph, edge_solnums_t>::type EdgeSolnumsMap;

typedef property_map<Graph, vertex_color_t>::type VertexColorMap;
typedef property_map<Graph, vertex_name_t>::type VertexNameMap;
  
typedef double Output;// which is the true geometric cost
typedef std::map<std::string, double> Input;// which consists of features
typedef std::map<Input, Output> Data;

static const std::string TRAJECTORY_FILTER = "/trajectory_filter_server/filter_trajectory_with_constraints";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const size_t NUM_PLANNING_ATTEMPTS = 1;
static const float ALLOWED_PLANNING_TIME = 0.1*60.;

static boost::mt19937 g_rng( static_cast<unsigned int>(std::time(0)) );

EdgeWeightMap g_edge_weight_map;
EdgeFlagMap g_edge_flag_map;
VertexColorMap g_vertex_color_map;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Helper Classes for graph operations ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename EdgeFlagMap>
class BestSolutionEdgeFilter
{
public:
  BestSolutionEdgeFilter() 
  { }
    
  BestSolutionEdgeFilter(EdgeFlagMap edge_flag_map)
    : edge_flag_map_(edge_flag_map) 
  { }
  
  template <typename Edge>
  bool 
  operator()(const Edge& e) const 
  {
    return (edge_flag_map_[e]==BEST_SOLUTION);
  }
  
private:
  EdgeFlagMap edge_flag_map_;
};

template <typename EdgeSolnumsMap>
class SolnumEdgeFilter
{
public:
  SolnumEdgeFilter() 
  { }
    
  SolnumEdgeFilter(EdgeSolnumsMap edge_solnums_map, size_t solnum)
    : edge_solnums_map_(edge_solnums_map), solnum_(solnum) 
  { }
  
  template <typename Edge>
  bool 
  operator()(const Edge& e) const 
  {
    std::vector<size_t>::iterator it;
    
    it = std::find(edge_solnums_map_[e].begin(), edge_solnums_map_[e].end(), solnum_);
    
    if( it!=edge_solnums_map_[e].end() )
      return true;
    else
      return false;
  }
  
private:
  EdgeSolnumsMap edge_solnums_map_;
  size_t solnum_;
};

template <typename EdgeFlagMap>
class PlannedEdgeFilter
{
public:
  PlannedEdgeFilter()
  { }
  
  PlannedEdgeFilter(EdgeFlagMap edge_flag_map)
    : edge_flag_map_(edge_flag_map) 
  { }
  
  template <typename Edge>
  bool 
  operator()(const Edge& e) const 
  {
    return (edge_flag_map_[e]==PLANNED) or (edge_flag_map_[e]==BEST_SOLUTION);
  }

private:
  EdgeFlagMap edge_flag_map_;  
};

struct VertexCopier  
{
  typedef boost::property_map< RawGraph, boost::vertex_name_t>::const_type RawGraphVertexNameMap;
  typedef boost::property_map< Graph, boost::vertex_name_t>::type GraphVertexNameMap;

  const RawGraph& raw_graph;
  RawGraphVertexNameMap raw_graph_vertex_name_map;
  GraphVertexNameMap graph_vertex_name_map;

  VertexCopier (const RawGraph& raw_graph_i, Graph& graph)
      : raw_graph(raw_graph_i)
      ,raw_graph_vertex_name_map(get(vertex_name, raw_graph_i))
      ,graph_vertex_name_map(get(vertex_name, graph))
  {
  }

  void 
  operator()(RawGraph::vertex_descriptor raw_graph_vertex, Graph::vertex_descriptor graph_vertex) const 
  {
    std::string name = get(raw_graph_vertex_name_map, raw_graph_vertex);
    put(graph_vertex_name_map, graph_vertex, name);
  }
};

struct EdgeCopier
{
  typedef boost::property_map< RawGraph, boost::edge_name_t>::const_type RawGraphEdgeNameMap;
  typedef boost::property_map< RawGraph, boost::edge_weight_t>::const_type RawGraphEdgeWeightMap;
  typedef boost::property_map< RawGraph, edge_flag_t>::const_type RawGraphEdgeFlagMap;

  typedef boost::property_map< Graph, boost::edge_name_t>::type GraphEdgeNameMap;
  typedef boost::property_map< Graph, boost::edge_weight_t>::type GraphEdgeWeightMap;
  typedef boost::property_map< Graph, edge_flag_t>::type GraphEdgeFlagMap;
  
  const RawGraph& raw_graph;
  RawGraphEdgeNameMap raw_graph_edge_name_map;
  RawGraphEdgeWeightMap raw_graph_edge_weight_map;
  RawGraphEdgeFlagMap raw_graph_edge_flag_map;
  GraphEdgeNameMap graph_edge_name_map;
  GraphEdgeWeightMap graph_edge_weight_map;
  GraphEdgeFlagMap graph_edge_flag_map;
  
  EdgeCopier(const RawGraph& raw_graph_i, Graph& graph)
    : raw_graph(raw_graph_i)
    ,raw_graph_edge_name_map(get(edge_name, raw_graph_i))
    ,raw_graph_edge_weight_map(get(edge_weight, raw_graph_i))
    ,raw_graph_edge_flag_map(get(edge_flag, raw_graph_i))    
    ,graph_edge_name_map(get(edge_name, graph))
    ,graph_edge_weight_map(get(edge_weight, graph))
    ,graph_edge_flag_map(get(edge_flag, graph))    
  {
  }
  
  void 
  operator()(RawGraph::edge_descriptor raw_graph_edge, Graph::edge_descriptor graph_edge) const 
  {
    std::string n = get(raw_graph_edge_name_map, raw_graph_edge);
    put(graph_edge_name_map, graph_edge, n);
    
    double w = get(raw_graph_edge_weight_map, raw_graph_edge);
    put(graph_edge_weight_map, graph_edge, w);
    
    size_t f = get(raw_graph_edge_flag_map, raw_graph_edge);
    put(graph_edge_flag_map, graph_edge, f);
  }
};

template<class Name, class Color>
class VertexPropWriter
{
public:
  VertexPropWriter(Name n, Color c)
    : name_(n), color_map_(c)
  {}
  
  template <class Vertex>
  void operator()(ostream& out, const Vertex& v) const
  {
    std::string color_str = "";
    if(color_map_[v]==color_traits<boost::default_color_type>::black())
    {
      color_str = ",color=\"green\"";
    }
    else if(color_map_[v]==color_traits<boost::default_color_type>::gray())
    {
      color_str = ",color=\"magenta\"";
    }
    
    out << "["
        << "label=\"" << name_[v] << "\",fontsize=\"10\"" 
        << color_str 
        << "]";
  }
private:
  Name name_;
  Color color_map_;
};

struct GraphPropWriter 
{
  void
  operator()(std::ostream& out) const 
  {
    out << "graph [bgcolor=lightgrey]" << std::endl;
  }
};


template <typename EdgeWeightMap, typename EdgeNameMap, typename EdgeFlagMap>
class EdgePropWriter  
{
public:
  EdgePropWriter (EdgeWeightMap edge_weight_map, EdgeNameMap edge_name_map, EdgeFlagMap edge_flag_map) 
    : edge_weight_map_(edge_weight_map)
    , edge_name_map_(edge_name_map)
    , edge_flag_map_(edge_flag_map)
  {}
  
  template <class Edge>
  void 
  operator()(ostream &out, const Edge& e) const 
  {
    std::string color_str = "";
    if(edge_flag_map_[e]==PLANNED)
    {
      color_str = ",color=\"green\"";
    }
    else if(edge_flag_map_[e]==PLANNED_BUT_FAILURE)
    {
      color_str = ",color=\"red\"";
    }
    else if(edge_flag_map_[e]==BEST_SOLUTION)
    {
      color_str = ",color=\"blue\"";
    }

    out << "["
    << "label=\"" << edge_name_map_[e] << "\\" << "n" << edge_weight_map_[e] << "\",fontsize=\"10\""
    << color_str 
    << "]";
  }
private:
  EdgeWeightMap edge_weight_map_;
  EdgeNameMap edge_name_map_;  
  EdgeFlagMap edge_flag_map_;
};

template <typename OutGraphType>
class DfsVisitor: public default_dfs_visitor
{
public:
  DfsVisitor(std::vector< std::list<Edge> >* sol_edge_lists)
    :sol_edge_lists_(sol_edge_lists)
  {}
  
  template < typename Edge, typename Graph >
  void 
  examine_edge(Edge e, const Graph& g) //const
  {
//    std::cout << "examine_edge( (" << boost::source(e, g) << "," << boost::target(e, g) << ") )" << std::endl;
  
    std::list<typename Graph::edge_descriptor> sol_edges;

    if(target(e,g)==TidyHome)
    {
      Edge e_loop = e;
      for(;;)
      {
        sol_edges.push_front(e_loop);
              
        typename Graph::vertex_descriptor s = source(e_loop,g);
        
        if(s==MessyHome)
        {
          break;
        }
        else
        {
          typename graph_traits<Graph>::in_edge_iterator i,j;
          tie(i,j) =  in_edges(s, g);
          e_loop = *i;
        }
      }
//      cout << "sol_edges.size()= " << sol_edges.size() << endl;

      sol_edge_lists_->push_back(sol_edges);
     }// End of:if(target(e,g)==TidyHome)
  }
private:  
  std::vector< std::list<Edge> >* sol_edge_lists_;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Utility Classes for A* search operations ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct FoundGoalSignal {}; // exception for termination

struct NoGeoPlanningSignal{};// for if there is no symbollically and geometrically feasible manipulation plan.

template <class Graph, class CostType>
class GeoCostHeuristic:public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  GeoCostHeuristic(Vertex goal)
    :goal_(goal)
  {}
  
  CostType 
  operator()(Vertex u)
  {
//    cout << "in h()" << endl;
    return 0;
  }
private:
  Vertex goal_;
};

template <typename Edge>
class GeoPlanningCmd
{
public:
  GeoPlanningCmd(Edge e)
    :e_(e)
  {
  }
  
  Edge
  edge()
  {
    return e_;
  }
  
private:  
  Edge e_;
};

template <class Vertex, class Edge>
class AstarVisitor: public boost::default_astar_visitor
{
public:
  AstarVisitor(Vertex goal)
    : m_goal(goal) 
  { }
  
  template <class Graph>
  void 
  examine_vertex(Vertex u, Graph& g) 
  {
    // Throw something if this examined vertex is the target of a geometrically INfeasible edge whose flag is 2
    typename graph_traits<Graph>::in_edge_iterator i,j;
    tie(i,j) =  in_edges(u, g);
    
    typedef typename property_map<Graph, edge_flag_t>::const_type FlagMap;// _must_ be ::const_type
    FlagMap edge_flag_map = get(edge_flag, g);
    
    if(edge_flag_map[*i] == PLANNED_BUT_FAILURE)
    {
      throw NoGeoPlanningSignal();
    }
    
    typedef typename property_map<Graph, vertex_color_t>::const_type ConstVertexColorMap;// _must_ be ::const_type
//    ConstVertexColorMap color_map = get(vertex_color, g);// TODO: have no idea why this leads to a compile error
  
    g_vertex_color_map[u] = color_traits < default_color_type>::black();//TODO Why does this make we can get vertex_color after returning from A*
    
    if(u == m_goal)
      throw FoundGoalSignal();
  }
  
  template <class Graph>
  void
  discover_vertex(Vertex u, Graph& g)
  {
    g_vertex_color_map[u] = color_traits < default_color_type>::gray();//TODO Why does this make we can get vertex_color after returning from A*
  }
  
  template<class Graph>
  void
  examine_edge(Edge e, Graph& g)
  {
//    std::cout << "examine_edge( (" << boost::source(e, g) << "," << boost::target(e, g) << ") )" << std::endl;

    typedef typename property_map<Graph, edge_flag_t>::const_type FlagMap;// _must_ be ::const_type
    FlagMap flag_map = get(edge_flag, g);

    if(flag_map[e]==NOT_YET)
    {
      throw GeoPlanningCmd<Edge>(e);
    }
  }
  
private:
  Vertex m_goal;
};

struct GeoPlanningCost
{
  GeoPlanningCost()
    :process(1.), result(1.), w_p(1.), w_r(1.)//TODO consider these init values
  {}
  
  double process;
  double result;
  
  double w_p;
  double w_r;
  
  double
  total()
  {
    return (w_p*(process/result)) + (w_r*(result/process));
  }
};

class PlannerManager
{
public:
//! A constructor.
/*!
  ...
*/
PlannerManager(ros::NodeHandle& nh):
  nh_(nh)
{
  ROS_INFO("Initializing the planner_master. Please wait ...");

  online_ = false;// Set the default
  if( !ros::param::get("/is_online", online_) )
    ROS_WARN("Can not get /is_online, use the default value (=false) instead");
  
  planner_manager_path_ = ".";
  if( !ros::param::get("/planner_manager_path", planner_manager_path_) )
    ROS_WARN("Can not get /task_planner_path, use the default value instead");
      
  if(online_)
    ros::service::waitForService("control_hand");
  
  ros::service::waitForService("plan_task");  
  
  ros::service::waitForService("plan_grasp");

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  set_planning_scene_diff_client_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);

  ros::service::waitForService(TRAJECTORY_FILTER);
  filter_trajectory_client_ = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER);
  
  motion_plan_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("motion_plan", 1000);
      
  collision_object_pub_ = nh_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  att_collision_object_pub_ = nh_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  ros::Duration(2.0).sleep();

  set_tidy_config();
  set_hiro_home_joint_state();
  
  if( !online_ )  
    goto_home_pose();
  
  ROS_INFO("planner_master: Up and Running");
}
//! A destructor.
/*!
  The destructor does nothing.
*/
~PlannerManager()
{
}

bool
is_messy_config_set()
{
  if( messy_config_.empty() )
    return false;
  else
    return true;
}

void 
collision_object_cb(const arm_navigation_msgs::CollisionObject::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%s]", msg->id.c_str());
  
  if( !strcmp(msg->id.c_str(), "table")
      or !strcmp(msg->id.c_str(), "wall")
      or !strcmp(msg->id.c_str(), "vase")
    )
  {
    return;
  }
  
  messy_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );

  // Avoid duplication TODO use std::find()
  bool is_new = true;
  size_t ith = 0;
  
  for(size_t i=0; i<messy_config_.size(); ++i)
  {
    if( !strcmp(msg->id.c_str(), messy_config_.at(i).id.c_str()) )
    {
      is_new = false;
      ith = i;
      break;
    }
  }
  
  if(is_new)
    messy_config_.push_back(*msg);
  else
  {
    messy_config_.at(ith).poses.at(0) = msg->poses.at(0);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////------CORE PUBLIC INTERFACE--------////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
clear_messy_config()
{
  messy_config_.clear();
}
//! Extract the true geometric planning cost of a node in a solution graph
/*!
  Note the the gpl here means the forward cost, from this node to the goal node.

  \param &v the input vertex
  \param &g the solution graph
  \param *out the true geo planning cost (forward)
*/
template<typename GraphType>
bool
extract_gpl(const typename GraphType::vertex_descriptor& v, const GraphType& g, Output* out)
{
  // Initialize
  *out = 0.;
  
  // From this vertices get the out_edge till the target of the out_edge is the goal
  typename graph_traits<GraphType>::out_edge_iterator oe_i,oe_j;
  tie(oe_i,oe_j) =  out_edges(v, g);// Assume that oe_i=oe_j
  
  typename GraphType::edge_descriptor e;
  e = *oe_i;  
  
  for( ; ; )// TODO make this recursive or use dfs_visit()
  {
    *out += get(edge_weight, g, e); 
    
    typename GraphType::vertex_descriptor t;
    t = target(e, g);
    
    if(t!=TidyHome)
    {
      tie(oe_i,oe_j) =  out_edges(t, g);// Assume that oe_i=oe_j
      
      e = *oe_i;
    }
    else
    {
      break;
    }
  }
  
  return true;
}
//! Extract the feature of a node in a solution graph
/*!
  More ...

  \param &v the input vertex
  \param &g the solution graph
  \param *f a map that maps feature name and its value
*/
template<typename GraphType>
bool
extract_features(const typename GraphType::vertex_descriptor& v, const GraphType& g, Input* f)
{
  // Symbolic Feature::
  // From this vertices get the out_edge till the target of the out_edge is the goal
  typename graph_traits<GraphType>::out_edge_iterator oe_i,oe_j;
  tie(oe_i,oe_j) =  out_edges(v, g);// Assume that oe_i=oe_j
  
  typename GraphType::edge_descriptor e;
  e = *oe_i;  
  
  double idx = 0.;
  for( ; ; )// TODO make this recursive or use dfs_visit()
  {
    idx = idx + 1.;// Note that the zero (double) value is reserved for a situation when a certain HLA name is not there.  
    
    std::string label;
    label = get(edge_name, g, e);
    
    (*f)[label] = idx;
    
    typename GraphType::vertex_descriptor t;
    t = target(e, g);
    
    if(t!=TidyHome)
    {
      tie(oe_i,oe_j) =  out_edges(t, g);// Assume that oe_i=oe_j
      
      e = *oe_i;
    }
    else
    {
      break;
    }
  }
  
  // Geometric Feature:: pose
  // Determine which object are in messy_spot or messy_spot by parsing the vertex name
  typedef typename property_map<GraphType, vertex_name_t>::type VertexNameMapHere;
  typename property_traits< VertexNameMapHere >::value_type v_name;
  v_name = get(vertex_name, g, v);// e.g CAN1[CAN2.CAN3.]

  std::vector<std::string> v_name_parts;
  boost::split( v_name_parts, v_name, boost::is_any_of("[") );// e.g from "CAN1[CAN2.CAN3.]" to "CAN1" and "CAN2.CAN3.]"

  std::vector<std::string> tidied_obj_ids;
  if( v_name_parts.size() > 1 )// DO not do this if v_name="MessyHome". Note that v_name="MessyHome" yields v_name_parts.size() = 1
  {
    boost::split( tidied_obj_ids, v_name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to CAN2 and CAN3 and ]
    tidied_obj_ids.erase(tidied_obj_ids.end()-1);//remove a "]"
  }
  
  for(std::map<std::string, arm_navigation_msgs::CollisionObject>::const_iterator i=messy_cfg_.begin(); i!=messy_cfg_.end(); ++i)
  {
    std::vector<std::string>::iterator found_it;
    found_it = std::find(tidied_obj_ids.begin(), tidied_obj_ids.end(), i->first);

    arm_navigation_msgs::CollisionObject obj;    

    if(found_it==tidied_obj_ids.end())
    {
      obj = tidy_cfg_[i->first];
    }
    else
    {
      obj = messy_cfg_[i->first];    
    }
    
    (*f)[std::string(i->first+".x")] = obj.poses.at(0).position.x;
    (*f)[std::string(i->first+".y")] = obj.poses.at(0).position.y;
    (*f)[std::string(i->first+".z")] = obj.poses.at(0).position.z;
    (*f)[std::string(i->first+".qx")] = obj.poses.at(0).orientation.x;
    (*f)[std::string(i->first+".qy")] = obj.poses.at(0).orientation.y;
    (*f)[std::string(i->first+".qz")] = obj.poses.at(0).orientation.z;
    (*f)[std::string(i->first+".qw")] = obj.poses.at(0).orientation.w;
  }
  
//  cout << "--------------" << endl;
//  for(Input::const_iterator j=f->begin(); j!=f->end(); ++j )
//    cout << (*j).first << "= " << (*j).second << endl;
//  cout << endl;
  
  return true;
}
//! Collect the learning(training) data from all feasible path
/*!
  May use dfs.
  Thus, the number of training samples is ...
  
  \return whether successful
*/
bool
collect()
{
  // Holds all training data
  std::string data_file_path = planner_manager_path_ + "/data/data.csv";
  
  std::ofstream data_file;
  data_file.open(data_file_path.c_str(), std::ios_base::app);
  
  std::map< Input, Output> data;

  // Extract all solutions!
  if( !extract_solutions() )
    return false;
  
  // Iterate all solutions, the index start from 1
  for(size_t solnum=1; ros::ok() ; ++solnum)
  {
    // Filter only edges that are flagged PLANNED
    SolnumEdgeFilter<EdgeSolnumsMap> edge_filter( get(edge_solnums, g_), solnum );
    typedef filtered_graph< Graph, SolnumEdgeFilter<EdgeSolnumsMap> > FilteredGraph;

    FilteredGraph filtered_g(g_, edge_filter);
   
    // Extract Feature for all vertices in the filtered graph but the last one
    boost::graph_traits<FilteredGraph>::edge_iterator ei, ei_end;
    boost::tie(ei,ei_end) = edges(filtered_g);
    
    if(ei == ei_end)
      break;
      
    for( ; ei!=ei_end; ++ei)// Excluding the last vertex, this will be done in the extract_feature()
    {
      FilteredGraph::vertex_descriptor s;
      s = source(*ei, filtered_g);
      
      Input* in = new Input();
      extract_features<FilteredGraph>(s, filtered_g, in);
      
      Output* out = new Output();
      extract_gpl<FilteredGraph>(s, filtered_g, out);
      
      data[*in] = *out;
    }
  }
  
  // TODO make length of input data uniform! mainly beacause HLA/sym features; By putting zero if actually that certain feature does not exist
  // Gather all feature ids
  std::set<std::string> feature_ids;
  for(std::map<Input, Output>::const_iterator data_it=data.begin(); data_it!=data.end(); ++data_it)
  {
    for(Input::const_iterator j=data_it->first.begin(); j!=data_it->first.end(); ++j )
    {
      feature_ids.insert(j->first);
    }
  }
  
  // Convert the set be converted to a vector to ensure the ordering
  std::vector<std::string> feature_ids_vec;
  for(std::set<std::string>::iterator it=feature_ids.begin(); it!=feature_ids.end(); ++it)
  {
    feature_ids_vec.push_back(*it);
  }
  
  for(std::vector<std::string>::iterator feature_ids_it=feature_ids_vec.begin(); feature_ids_it!=feature_ids_vec.end(); ++feature_ids_it)
  {
    cout << *feature_ids_it << ",";
    data_file << *feature_ids_it << ",";
  }
  cout << "OUT";
  data_file << "OUT";
  cout << endl;
  data_file << std::endl;
  
  // Write  
  for(std::map<Input, Output>::const_iterator data_it=data.begin(); data_it!=data.end(); ++data_it)
  {
//    cerr << "=========================================================" << endl;
//    for(Input::const_iterator j=data_it->first.begin(); j!=data_it->first.end(); ++j )
//      cout << (*j).first << "= " << (*j).second << "," << endl;
//    cout << endl;
      
    // Write the input
    for(std::vector<std::string>::iterator feature_ids_it=feature_ids_vec.begin(); feature_ids_it!=feature_ids_vec.end(); ++feature_ids_it)
    {
      // Find whether this feature exists (has value >0)
      Input local_in;
      local_in = data_it->first;
      
      Input::iterator it;
      it = local_in.find(*feature_ids_it);
      
      if(it==local_in.end())
      {
        cout << 0. << ",";
        data_file << 0. << ",";
      }
      else
      {
        cout << it->second << ",";
        data_file << it->second << ",";
      }
    }
    
    // Write the output
    cout << data_it->second;
    data_file << data_it->second;
    cout << endl;
    data_file << std::endl;
  }  
  
  data_file.close();
  
  return true;
}
//! Plan manipulation plans.
/*!
  The output is of high quality (optimal) and guaranteed to be symbollically and geometrically feasible.
*/
void
plan()
{
  // Create task plan space encoded in a task motion graph
  plan_symbollically();
  
  // Searching over task plan space, within which a geometric planner is called to validate eash symbollically feasible task plan.
  Vertex start = MessyHome;
  Vertex goal = TidyHome;  
  
  // For utilities through out this function
  property_map<Graph, vertex_name_t>::type vertex_name_map;
  vertex_name_map = get(vertex_name, g_);
  
  property_map<Graph, edge_name_t>::type edge_name_map;
  edge_name_map = get(edge_name, g_);
  
  // Start searching
  vector<Graph::vertex_descriptor> p(num_vertices(g_));
  vector<double> d(num_vertices(g_));

  bool path_found = false;
  while(ros::ok())
  {
    try 
    {
      astar_search(g_, 
                   start,
                   GeoCostHeuristic<Graph, double>(goal),
                   predecessor_map(&p[0]).distance_map(&d[0]).visitor(AstarVisitor<Vertex, Edge>(goal))
                  );
    }
    catch(GeoPlanningCmd<Edge> geo_planning_cmd)
    {
      // Plan geometrically now!
      Edge e;
      e = geo_planning_cmd.edge();
      
      plan_geometrically(e);
      
      // Reset vertex_color on the entire graph, TODO is this necessary?
      boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      for(boost::tie(vi,vi_end) = vertices(g_); vi!=vi_end; ++vi)
      {
        g_vertex_color_map[*vi] = color_traits<boost::default_color_type>::white();
      }
      
      continue;
    }
    catch(FoundGoalSignal fg) 
    { // found a path to the goal
      path_found = true; 
      break;
    }
    catch(NoGeoPlanningSignal np)
    {
      path_found = false;
      break;
    }
  }// End of:  while(true)

  if(path_found)  
  {
    list<Vertex> man_plan_vertices_tmp;
    for(Vertex v = goal;; v = p[v]) 
    {
      man_plan_vertices_tmp.push_front(v);// Note that p contains a backtracking record
      
      if(p[v] == v)
        break;
    }
      
    vector<Vertex> man_plan_vertices;
    man_plan_vertices.push_back(start);
    
    list<Vertex>::iterator mpv_i = man_plan_vertices_tmp.begin();
    cout << "Manipulation plan (Vertices)= " << vertex_name_map[start];
    for(++mpv_i; mpv_i != man_plan_vertices_tmp.end(); ++mpv_i)
    {
      man_plan_vertices.push_back(*mpv_i);
      cout << " -> " << vertex_name_map[*mpv_i];
    }
    cout << endl;
    
    cout << "ManipulationPlanCost= " << d[goal] << endl;
        
    std::vector<Edge> man_plan_edges;
    for(std::vector<Vertex>::const_iterator i=man_plan_vertices.begin(); i!=man_plan_vertices.end()-1; ++i)
    {
      Edge e;
      bool connected;

      tie(e, connected) = edge(*i, *(i+1), g_);

      property_traits<EdgeFlagMap>::value_type e_flag;
      e_flag = BEST_SOLUTION;
      
      put(edge_flag, g_, e, e_flag);      

      man_plan_edges.push_back(e);
    }
    
    cout << "Manipulation plan (Edges)= ";
    for(std::vector<Edge>::const_iterator i=man_plan_edges.begin(); i!=man_plan_edges.end(); ++i)
    {
      cout << edge_name_map[*i] << " -> ";
    }
    cout << endl;
  }
  else
  {
    cout << "NO path" << endl;  
  }
  
  // Write dot file final
  property_map<Graph, vertex_color_t>::type color_map;
  color_map = get(vertex_color, g_);// TODO have no idea why this can capture vertex_color value, while actually it should be obtained from g_vertex_color_map
  
  std::string dot_path = planner_manager_path_ + "/tmg/tmg.dot";
  
  ofstream dotfile_final;
  dotfile_final.open(dot_path.c_str());
  write_graphviz( dotfile_final, g_
                , VertexPropWriter< property_map<Graph, vertex_name_t>::type,property_map<Graph, vertex_color_t>::type >(vertex_name_map, color_map)
                , EdgePropWriter<EdgeWeightMap, EdgeNameMap, EdgeFlagMap>(g_edge_weight_map, edge_name_map, g_edge_flag_map)
                , GraphPropWriter()
                );  
  dotfile_final.close();
}
//! Commit the best manipulation plan
/*!
 If any...
*/
bool
commit()
{
  BestSolutionEdgeFilter<EdgeFlagMap> edge_filter(get(edge_flag, g_));
  typedef filtered_graph< Graph, BestSolutionEdgeFilter<EdgeFlagMap> > FilteredGraph;

  FilteredGraph man_plan_g(g_, edge_filter);

  std::string dot_path = planner_manager_path_ + "/tmg/committed_tmg.dot";
  
  ofstream dotfile_final;
  dotfile_final.open(dot_path.c_str());
  write_graphviz( dotfile_final, man_plan_g_
//                , VertexPropWriter< property_map<FilteredGraph, vertex_name_t>::type,property_map<FilteredGraph, vertex_color_t>::type >(vertex_name_map, color_map)
//                , EdgePropWriter<EdgeWeightMap, EdgeNameMap, EdgeFlagMap>(g_edge_weight_map, edge_name_map, g_edge_flag_map)
//                , GraphPropWriter()
                );  
  dotfile_final.close();
  
  bool success = false;  
  graph_traits<FilteredGraph>::edge_iterator ei, ei_end;

  for (boost::tie(ei, ei_end) = edges(man_plan_g); ei != ei_end; ++ei)
  {
    std::string e_name = get(edge_name, g_, *ei);
    ROS_INFO_STREAM("Committing: " << e_name);
    
    // Parse edge name to determine the operator and its args
    std::vector<std::string> e_name_parts;  
    boost::split( e_name_parts, e_name, boost::is_any_of("_") );
    
    // Call either motion or grasp
    if( !strcmp(e_name_parts.at(0).c_str(), "TRANSIT") or !strcmp(e_name_parts.at(0).c_str(), "TRANSFER") )
    {
      success = commit_motion( get(edge_impl, g_, *ei).motion_path );
    }
    else if( !strcmp(e_name_parts.at(0).c_str(), "GRASP") )
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
      object_it = get_object(e_name_parts.at(2), messy_config_);
      
      if(object_it == messy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << e_name_parts.at(2) << " from the messy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
      
      success = commit_grasp( object );
    }
    else if( !strcmp(e_name_parts.at(0).c_str(), "UNGRASP") )
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
      object_it = get_object(e_name_parts.at(2), tidy_config_);
      
      if(object_it == tidy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << e_name_parts.at(2) << " from the tidy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
      
      success = commit_ungrasp( object );
    }
  }
  
  return success;
}

private:
//! A task motion graph variable
/*!
  Task plan space is encoded here.
*/
Graph g_; 

//! A task motion graph variable
/*!
  Task plan space is encoded here.
*/
Graph man_plan_g_; 

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

//! Contain a messy config as an input for the manipulation planner.
/*!
  Holds the answer for WHAT? including shapes and WHERE?
*/
std::vector<arm_navigation_msgs::CollisionObject> messy_config_;
std::map<std::string, arm_navigation_msgs::CollisionObject> messy_cfg_;

//! Contain a tidy config as an input for the manipulation planner.
/*!
 Holds the answer for WHAT? including shapes and WHERE?
 Future works should include compromisable tidy configs
*/
std::vector<arm_navigation_msgs::CollisionObject> tidy_config_;
std::map<std::string, arm_navigation_msgs::CollisionObject> tidy_cfg_;

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

ros::ServiceClient filter_trajectory_client_;

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

//! A helper variable
/*!
  More ...
*/
sensor_msgs::JointState hiro_home_joint_state_;

//! A helper variable
/*!
  More ...
*/
std::string planner_manager_path_;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////------CORE PLANNERS--------////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! Brief ...
/*!
  This function create the task motion graph (TMG), which stored in a private variable named: g_.
  Note that the result of calling task_plan service is stored in .tmg file, which is the base for creating the TMG g_.
  
  \return whether successful
*/
bool
plan_task()
{
  // Make a request to plan_task srv
  task_planner::PlanTask::Request plan_task_req;
  task_planner::PlanTask::Response plan_task_res;
  
  for(size_t i=0; i<messy_config_.size(); ++i)
  {
    if( !strcmp(messy_config_.at(i).id.c_str(),"unmovable_object"))
      continue;
  
    plan_task_req.objects.push_back(messy_config_.at(i));
  }
  
  ros::service::waitForService("plan_task"); 
  ros::ServiceClient plan_task_client;
  plan_task_client = nh_.serviceClient<task_planner::PlanTask>("plan_task");  

  if (plan_task_client.call(plan_task_req, plan_task_res))
  {
    ROS_DEBUG("Succeeded to call plan_task service");
  }
  else
  {
    ROS_ERROR("Failed to call plan_task service");
    return false;
  }
  
  // Create the task motion graph (from a file) 
  std::string dat_path = planner_manager_path_ + "/tmg/tmg.dat";
  RawGraph raw_g; 
  
  std::ifstream graph_data(dat_path.c_str());
  graph_data >> read( raw_g );  
  
  boost::copy_graph(raw_g
                    , g_
                    , boost::vertex_copy(VertexCopier (raw_g, g_))
                    . edge_copy(EdgeCopier(raw_g, g_))
                   );
  
    // Sync the global copy of edge_weight  
  g_edge_weight_map = get(edge_weight, g_);
  g_edge_flag_map = get(edge_flag, g_);
  g_vertex_color_map = get(vertex_color, g_);
  
  return true;
}
//! Brief ...
/*!
  More ...
  Assuming that the motion planner is an oracle i.e. always returns the best motion plan for given a pair of start and goal states
  Note: the planner does not put any values at motion_plan.joint_trajectory.points.at(0).velocities
  motion_plan.joint_trajectory.points.at(0).velocities.size() is always zero here.
  
  \param start_state
  \param goal_state
  \param *path
  \param *cost
  \return whether successful
*/
bool
plan_motion(const sensor_msgs::JointState& start_state, const sensor_msgs::JointState& goal_state, arm_navigation_msgs::RobotTrajectory* motion_path, GeoPlanningCost* cost=0)
{
  while(!ros::service::waitForService("ompl_planning/plan_kinematic_path", ros::Duration(1.0))) {
    ROS_INFO_STREAM("Waiting for requested service " << "ompl_planning/plan_kinematic_path");
  }
  
  ros::ServiceClient planning_client = nh_.serviceClient<arm_navigation_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");

  arm_navigation_msgs::GetMotionPlan::Request req;  
  arm_navigation_msgs::GetMotionPlan::Response res;
  
  req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
  req.motion_plan_request.group_name = "rarm";
  req.motion_plan_request.num_planning_attempts = NUM_PLANNING_ATTEMPTS;
  req.motion_plan_request.allowed_planning_time = ros::Duration(ALLOWED_PLANNING_TIME);
  req.motion_plan_request.planner_id= std::string("");
  
  req.motion_plan_request.goal_constraints.joint_constraints.resize(goal_state.name.size());
  
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
  
  // TODO Due to probabilistically complete, we should call the plan as many as possible
  if (planning_client.call(req, res))
  {
    if (res.trajectory.joint_trajectory.points.empty())
    {
      ROS_WARN("Motion planner was unable to plan a path to goal");

      cost->result += 0.;// Note that the result depends only on the path length
      cost->process = 10.;// TODO make this not a magic number

      return false;
    }
    else
    {
      ROS_DEBUG("Motion planning succeeded");
    
      trajectory_msgs::JointTrajectory filtered_trajectory;
      filter_path(res.trajectory.joint_trajectory, req, &filtered_trajectory);
      motion_plan_pub_.publish(filtered_trajectory);
      
      // Visualize the path
//      motion_plan_pub_.publish(res.trajectory.joint_trajectory);
     motion_plan_pub_.publish(filtered_trajectory);
      
      // Put the response 
      *motion_path = res.trajectory;

      // Benchmark the path
      ros::ServiceClient benchmark_path_client = nh_.serviceClient<hiro_common::BenchmarkPath>("benchmark_motion_plan");

      hiro_common::BenchmarkPath::Request benchmark_path_req;
      hiro_common::BenchmarkPath::Response benchmark_path_res;

//      benchmark_path_req.trajectory = res.trajectory.joint_trajectory;
      benchmark_path_req.trajectory = filtered_trajectory;

      if (benchmark_path_client.call(benchmark_path_req, benchmark_path_res))
      {
        ROS_DEBUG("BenchmarkPath succeeded");
        cost->result += benchmark_path_res.length;// Note that the result depends only on the path length
        cost->process = 1.;
      }
      else
      {
        ROS_ERROR("BenchmarkPath service failed on %s",benchmark_path_client.getService().c_str());
        return false;
      }
      
      // Clear the path visualization
      trajectory_msgs::JointTrajectory empty_path;
      ros::Duration(0.1).sleep();// May be not necessary
      motion_plan_pub_.publish(empty_path);
      return true;
    }
  }
  else
  {
    ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
    return false;
  }
}

bool
plan_motion(const std::vector<sensor_msgs::JointState>& start_states, const std::vector<sensor_msgs::JointState>& goal_states, arm_navigation_msgs::RobotTrajectory* best_motion_path, GeoPlanningCost* best_cost=0)
{
  best_cost->result = std::numeric_limits<double>::max();// Initialize with a very long path!  

  size_t n_failure = 0;// For computing motion planning process cost

  // For suppressing the number of motion planning trials
  size_t min_n_success;
  min_n_success = (size_t)(0.05 * start_states.size()*goal_states.size());// TODO attempts should be picked randomly
  if (min_n_success < 1)
    min_n_success = 1;// At least, one attempt
  
  min_n_success = 1;// TODO remove me!
   
  size_t n_success = 0;
  
  for(std::vector<sensor_msgs::JointState>::const_iterator start_state_it=start_states.begin(); start_state_it!=start_states.end(); ++start_state_it)
  {
    for(std::vector<sensor_msgs::JointState>::const_iterator goal_state_it=goal_states.begin(); goal_state_it!=goal_states.end(); ++goal_state_it)
    {
      arm_navigation_msgs::RobotTrajectory motion_path;
      GeoPlanningCost cost;
      
      if( !plan_motion(*start_state_it, *goal_state_it, &motion_path, &cost) )
      {
        ROS_WARN("Motion planning failed, continue...");
        
        ++n_failure;
        
        continue;
      } 
      
      
      // Note here we determine the best motion path that depends only on cost.result value
      if( (cost.result < best_cost->result) )
      {
        best_cost->result = cost.result;
        best_cost->process = cost.process;
        *best_motion_path = motion_path;
        
        // TODO mark start and goal grasps that form the best path
      }
      
      ++n_success;
      
      if(n_success >= min_n_success)
        break;
    }
    if(n_success >= min_n_success)
    break;
  } 

  //Compute the process cost;
  double iter_cost;
  iter_cost = (double) n_failure / min_n_success * 100.;// in the range og [0,100] to suppress the ratio value
  best_cost->process += iter_cost;
        
  if( best_motion_path->joint_trajectory.points.empty() )
  {
    ROS_WARN("All motion planing attempts on all start-goal-state pairs: FAILED");
    //TODO should we penalize the process cost again!
    
    return false;
  }
  else
  {
    return true;
  }
}
//! Brief ...
/*!
  More ...
  This is for both grasp and ungrasp operations.
  
  \param object The collision object to be grasp or ungrasped.
  \param revert Whether the function has to revert the planning environment after grasp/ungrasp planning.
  \return All feasible grasp poses.
*/
bool
plan_grasp(arm_navigation_msgs::CollisionObject& object, std::vector<sensor_msgs::JointState>* grasp_poses, GeoPlanningCost* cost=0)
{
  // Set or reset the object in the planning_environment
  object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
  collision_object_pub_.publish(object);
  ros::Duration(2.0).sleep();

  if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    ROS_WARN("Can't get planning scene. Env can not be configured correctly");
  
  // Call the plan_grasp srv
  grasp_planner::PlanGrasp::Request plan_grasp_req;
  grasp_planner::PlanGrasp::Response plan_grasp_res; 
  
  plan_grasp_req.object = object;
  
  ros::service::waitForService("plan_grasp");
  ros::ServiceClient plan_grasp_client;
  plan_grasp_client = nh_.serviceClient<grasp_planner::PlanGrasp>("plan_grasp");
    
  if (plan_grasp_client.call(plan_grasp_req, plan_grasp_res))
  {
    ROS_DEBUG("Succeeded to call plan_grasp service");
    
    if( !plan_grasp_res.grasp_plans.empty() )
    {
      *grasp_poses = plan_grasp_res.grasp_plans;
      
      cost->process = plan_grasp_res.process_cost;// TODO get the process cost info from plan_grasp response
      cost->result = 1.;

      return true;
    }
    else
    {
      cost->process = plan_grasp_res.process_cost;
      cost->result = 1.;
      return false;    
    }
  }
  else
  {
    ROS_WARN("Failed to call plan_grasp service");
    return false;
  }
}

bool
plan_ungrasp(arm_navigation_msgs::CollisionObject& object, std::vector<sensor_msgs::JointState>* ungrasp_poses, GeoPlanningCost* cost=0)
{
  bool is_successful = false;
  
  is_successful = plan_grasp(object, ungrasp_poses, cost); 
  
  // Revert the particular object in the planning_environment to the messy_config_
  // Revert is for ungrasp planning, where the object passed here is from tidy_config_
  std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
  object_it = get_object(object.id, messy_config_);
      
  if(object_it == messy_config_.end())
    ROS_ERROR_STREAM("Can not find " << object.id << " from the messy_config_");
      
  object = *object_it;// Change object from tidy_config_ to messy_config_
      
  object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
  collision_object_pub_.publish(object);
  ros::Duration(2.0).sleep();

  if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
    ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    
  return is_successful;
}
//! Plan at symbolic level
/*!
  Do a calling to plan_task().
  
  \return whether successful
*/
bool
plan_symbollically()
{
  return plan_task();
}

//! Plan at geometric level
/*!
  This includes calling plan_grasp() and plan_motion().
  Also bencmarking the motion plan, which is the lowest level of a manipulation plan that determines the quality of a manipulation plan.
  
  \param *plan A pointer to a manipulation plan.
*/
bool
plan_geometrically(const Edge& e)
{
  // Call motion planning, note that grasp planning is called within motion planning
  property_traits<EdgeNameMap>::value_type e_name;
  e_name = get(edge_name, g_, e);

  std::vector<std::string> e_name_parts;  
  boost::split( e_name_parts, e_name, boost::is_any_of("_") );
  
  // TODO if this is  a GRASP then do nothing, because grasp planning is called within motion planning, although likely that the edge passed in is not GRASP/UNGRASP
  
  // Determine the start states for the motion planning by retrieving the implementation of the previous edge.
  Vertex s;
  s = source(e, g_);
    
  std::vector<sensor_msgs::JointState> start_states;
  if( !strcmp(get(vertex_name, g_, s).c_str(), "MessyHome") )
  {
    start_states.push_back(hiro_home_joint_state_);
  }
  else
  {
    // Obtain the start state from the previous edge's implementation, which the in_edges of the source vertex
    graph_traits<Graph>::in_edge_iterator ie_i,ie_j;// Assume that ie_i = ie_j because there is only one in-edge
    tie(ie_i,ie_j) =  in_edges(s, g_);
    
    property_traits<EdgeImplMap>::value_type impl;
    impl = get(edge_impl, g_, *ie_i);
    
    start_states = impl.grasp_poses;
  }
  
  // Determine the goal state for the motion planning by calling the grasp planner for implementing the next edge
  Vertex t;
  t = target(e, g_);
    
  std::vector<sensor_msgs::JointState> goal_states;
  
  if( !strcmp(get(vertex_name, g_, t).c_str(), "TidyHome") )
  {
    goal_states.push_back(hiro_home_joint_state_); 
  }
  else
  {
    // Set the planning environment so as to satisfy the init_state, i.e. by setting already-tidied-up object at tidy spot. Get the name and parse
    property_traits<VertexNameMap>::value_type t_name;
    t_name = get(vertex_name, g_, t);// e.g CAN1[CAN2.CAN3.]
    
    std::vector<std::string> t_name_parts;
    boost::split( t_name_parts, t_name, boost::is_any_of("[") );// e.g from "CAN1[CAN2.CAN3.]" to "CAN1" and "CAN2.CAN3.]"
    
    std::vector<std::string> tidiedup_object_ids;
    boost::split( tidiedup_object_ids, t_name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to CAN2 and CAN3 and ]
    tidiedup_object_ids.erase(tidiedup_object_ids.end()-1);//remove a "]"
  
    // Set planning environment
    for(std::vector<std::string>::const_iterator i=tidiedup_object_ids.begin(); i!=tidiedup_object_ids.end(); ++i)
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
      object_it = get_object(*i, tidy_config_);
      
      if(object_it == tidy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << *i << " from the tidy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
        
      object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
      collision_object_pub_.publish(object);
      ros::Duration(2.0).sleep();

      if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
        ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    }
     
     // Call grasp planner or ungrasp planner depending on out_edge of target vertex operator then UPDATE its edge implementation
    graph_traits<Graph>::out_edge_iterator oe_i,oe_j;// Note that oe stands for out_edge of the target vertex of edge e
    tie(oe_i,oe_j) =  out_edges(t, g_);// Assume that oe_i=oe_j
    
    property_traits<EdgeNameMap>::value_type out_edge_name;
    out_edge_name = get(edge_name, g_, *oe_i);

    std::vector<std::string> out_edge_name_parts;  
    boost::split( out_edge_name_parts, out_edge_name, boost::is_any_of("_") );
    
    std::string op = out_edge_name_parts.at(0);
    
    property_traits<EdgeImplMap>::value_type oe_impl;
    property_traits<EdgeFlagMap>::value_type oe_flag;
    GeoPlanningCost grasp_planning_cost;
  
    if( !strcmp(out_edge_name_parts.at(0).c_str(), "GRASP") )
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;// Not a const_iterator because its time stamp has to be updated!
      object_it = get_object(out_edge_name_parts.at(2), messy_config_);// Assume that GRASP is always at messy_spot
      
      if(object_it == messy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << out_edge_name_parts.at(2) << " from the messy_config_");
        return false;// TODO is this ok?
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
      
      if( plan_grasp(object, &goal_states, &grasp_planning_cost) )
      {
        oe_flag = PLANNED;
        oe_impl.grasp_poses = goal_states;
      }
      else
      {
        oe_flag = PLANNED_BUT_FAILURE;
      }
    }
    else if( !strcmp(out_edge_name_parts.at(0).c_str(), "UNGRASP") )
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;// Not a const_iterator because its time stamp has to be updated!
      object_it = get_object(out_edge_name_parts.at(2), tidy_config_);// Assume that GRASP is always at messy_spot
      
      if(object_it == tidy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << out_edge_name_parts.at(2) << " from the tidy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
      
      if( plan_ungrasp(object, &goal_states, &grasp_planning_cost) )
      {
        oe_flag = PLANNED;
        oe_impl.grasp_poses = goal_states;
      }
      else
      {
        oe_flag = PLANNED_BUT_FAILURE;
      }
    }
     
    put(edge_impl, g_, *oe_i, oe_impl);
    put(edge_flag, g_, *oe_i, oe_flag);
    put(edge_weight, g_, *oe_i, grasp_planning_cost.total());
    
    // Important to ...
    reset_collision_space();
  }

  // Set the planning environmet as specisified by the source vertex so as to do below motion planning, synch with the state in the source node
  property_traits<VertexNameMap>::value_type s_name;// the name of source vertex of edge e
  s_name = get(vertex_name, g_, s);// e.g "Grasped_CAN1[CAN2.CAN3.]"
  
  if( strcmp(s_name.c_str(), "MessyHome") )// If the s-name != "MessyHome"
  {
    std::vector<std::string> s_name_parts;
    boost::split( s_name_parts, s_name, boost::is_any_of("[") );// e.g from "Grasped_CAN1[CAN2.CAN3.]" to "Grasped_CAN1" and "CAN2.CAN3.]"

    std::vector<std::string> tidiedup_object_ids;
    boost::split( tidiedup_object_ids, s_name_parts.at(1), boost::is_any_of(".") );// e.g. from "CAN2.CAN3.]" to "CAN2" and "CAN3" and "]"
    tidiedup_object_ids.erase(tidiedup_object_ids.end()-1);//remove a "]"

    // Set planning environment
    for(std::vector<std::string>::const_iterator i=tidiedup_object_ids.begin(); i!=tidiedup_object_ids.end(); ++i)
    {
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
      object_it = get_object(*i, tidy_config_);
      
      if(object_it == tidy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << *i << " from the tidy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
        
      object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
      collision_object_pub_.publish(object);
      ros::Duration(2.0).sleep();

      if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
        ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    }
    
    std::vector<std::string> state_parts;
    boost::split( state_parts, s_name_parts.at(0), boost::is_any_of("_") );// e.g. from "Grasped_CAN1" to "Grasped" and "CAN1"
    
    if( !strcmp(state_parts.at(0).c_str(), "Released") )// If source state of edge e is RELEASED then the released_objects _must_ be in tidyspot, this is for TRANSIT
    {
      // Set the released object in the tidy spot
      std::vector<arm_navigation_msgs::CollisionObject>::const_iterator object_it;
      object_it = get_object(state_parts.at(1), tidy_config_);
      
      if(object_it == tidy_config_.end())
      {
        ROS_ERROR_STREAM("Can not find " << state_parts.at(1) << " from the tidy_config_");
        return false;
      }
      
      arm_navigation_msgs::CollisionObject object;
      object = *object_it;
        
      object.header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
      collision_object_pub_.publish(object);
      ros::Duration(2.0).sleep();

      if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) 
        ROS_WARN("Can't get planning scene. Env can not be configured correctly");
    }
    else if( !strcmp(state_parts.at(0).c_str(), "Grasped") )// If the sources state of edge e is GRASPED, the object _must_be grasped.
    {
      // Update planning_environment
      arm_navigation_msgs::AttachedCollisionObject att_object;
      
      att_object.link_name = "link_rhand_palm";
      //att_object.touch_links.push_back("r_gripper_palm_link");
      att_object.object.id = state_parts.at(1);
      att_object.object.header.frame_id = "link_rhand_palm";
      att_object.object.header.stamp = ros::Time::now();
      att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;    
      
      att_collision_object_pub_.publish(att_object);
      ros::Duration(2.0).sleep();
      
      ROS_INFO("waitForService(SET_PLANNING_SCENE_DIFF_NAME)");
      ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

      if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) {
        ROS_WARN("Can't get planning scene");
        return false;
      }
    }
  }// End of: if( strcmp(s_name.c_str(), "MessyHome") )// If the s-name != "MessyHome"
  
  // Call the motion planning now. Pick only the best motion path, TODO also MArk the best grasp plan
  arm_navigation_msgs::RobotTrajectory best_motion_path;
  GeoPlanningCost best_motion_planning_cost;

  property_traits<EdgeImplMap>::value_type e_impl;
  property_traits<EdgeFlagMap>::value_type e_flag;
  property_traits<EdgeWeightMap>::value_type e_weight;

  if( plan_motion(start_states, goal_states, &best_motion_path, &best_motion_planning_cost) )
  {
    e_impl.motion_path = best_motion_path; 
    e_flag = PLANNED; 
    e_weight = best_motion_planning_cost.total();
  }
  else
  {
    e_impl.motion_path = best_motion_path; 
    e_flag = PLANNED_BUT_FAILURE; 
    e_weight = best_motion_planning_cost.total();
  }  
  
  put(edge_weight, g_, e, e_weight);
  put(edge_impl, g_, e, e_impl);
  put(edge_flag, g_, e, e_flag);
  
  // Important to ...
  reset_collision_space();

  return true;  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////------PLAN EXECUTORS--------///////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! Commit a motion plan
/*!
  More...
  
  \param plan A motion plan, which is a trajectory (path)
  \return whether successful
*/
bool
commit_motion(const arm_navigation_msgs::RobotTrajectory& plan)
{
  ROS_DEBUG("Commit a motion_plan: BEGIN");
  
  // Visualize the plan
  motion_plan_pub_.publish(plan.joint_trajectory);

  // Sendding commmands to joint_controller
  hiro_control::MoveArm::Request req;
  hiro_control::MoveArm::Response res;

  req.robot_trajectory = plan;
    
  ros::ServiceClient commit_client = nh_.serviceClient<hiro_control::MoveArm>("move_arm");
  
  if (commit_client.call(req, res))
    ROS_DEBUG("Motion planning succeeded");
  else
  {
    ROS_ERROR("Motion planning service failed on %s",commit_client.getService().c_str());
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
commit_grasp(const arm_navigation_msgs::CollisionObject& object)
{
  // Update planning_environment
  arm_navigation_msgs::AttachedCollisionObject att_object;
  
  att_object.link_name = "link_rhand_palm";
  //att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.object.id = object.id;
  att_object.object.header.frame_id = "link_rhand_palm";
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;    
  
  att_collision_object_pub_.publish(att_object);
  ros::Duration(2.0).sleep();
  
  ROS_INFO("waitForService(SET_PLANNING_SCENE_DIFF_NAME)");
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

  if( !set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_) ) {
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
  
  ROS_INFO("Should have been grasped");
  return true;  
}
//! Commit the ungrasp
/*!
  More...
  
  \param object The will-be-released object
  \return whether successful
*/
bool
commit_ungrasp(const arm_navigation_msgs::CollisionObject& object)
{
  // Update planning_environment
    
  arm_navigation_msgs::AttachedCollisionObject att_object;
  
  att_object.link_name = "link_rhand_palm";
  //att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.object.id = object.id;
  att_object.object.header.frame_id = "link_rhand_palm";
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;    
  
  att_collision_object_pub_.publish(att_object);
  ros::Duration(2.0).sleep();
  
  ROS_INFO("waitForService(SET_PLANNING_SCENE_DIFF_NAME)");
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

  if(!set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_)) {
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
  {
    ros::Duration(.5).sleep();
  }
  
  ROS_INFO("Should have been ungrasped");
  return true;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////------HELPER (UTILITY)--------/////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! Extract all possible solutions from the graph resulted from A* search
/*!
  This function updates the edge_solnums properties of edges.
  The index begins at 1, because the value 0 is reserved for the best solution, although this does not check for that up to now.
  \param *sol_gs A pointer to a vector of solution graphs
  \return whether successful
*/
bool
extract_solutions()
{
  // Filter only edges that are flagged PLANNED
  PlannedEdgeFilter<EdgeFlagMap> edge_filter(get(edge_flag, g_));
  typedef filtered_graph< Graph, PlannedEdgeFilter<EdgeFlagMap> > FilteredGraph;

  FilteredGraph filtered_g(g_, edge_filter);
  
  // Do dfs on the filtered_graph
  std::vector< std::list<Edge> > sol_edge_lists;
  
  DfsVisitor< Graph > vis(&sol_edge_lists);
  depth_first_search(filtered_g, visitor(vis));
  
  if( sol_edge_lists.empty() )
    return false;

  // Mark the sol_num property of the graph g_
  for(std::vector< std::list<Edge> >::const_iterator i=sol_edge_lists.begin(); i!=sol_edge_lists.end(); ++i)
  {
    for(std::list<Edge>::const_iterator j=i->begin(); j!=i->end(); ++j)
    {
      size_t solnum = i-sol_edge_lists.begin()+1; // Plus one because sol_num = 0 is reserved to the best solution
      
      std::vector<size_t> solnums;
      
      // Get the current solnums
      solnums = get(edge_solnums, g_, *j);
      
      // Update the solnums
      solnums.push_back(solnum);
      put(edge_solnums, g_, *j, solnums);
    }
  }
  
  return true;
}
//! Filter the motion path
/*!
  More ...
  \param trajectory_in trajectory_out  
  \return whether successful
*/
bool
filter_path(const trajectory_msgs::JointTrajectory& trajectory_in, const arm_navigation_msgs::GetMotionPlan::Request& get_motion_plan_req, trajectory_msgs::JointTrajectory* trajectory_out)
{
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;

//  req.trajectory.joint_names = trajectory_in.joint_names;
//  req.trajectory.points = trajectory_in.points;
  req.trajectory = trajectory_in;
  req.group_name = "rarm";
  req.start_state =get_motion_plan_req.motion_plan_request.start_state;
  req.path_constraints = get_motion_plan_req.motion_plan_request.path_constraints;
  req.goal_constraints = get_motion_plan_req.motion_plan_request.goal_constraints;
  req.allowed_time = ros::Duration(3.0);

  if(filter_trajectory_client_.call(req,res))
  {
    *trajectory_out = res.trajectory;
    return true;
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed.");
    return false;
  }
}
//! Reset the collision space or planning environment.
/*!
  That is to the messy config.
  
  \return whether successful
*/
bool
reset_collision_space()
{
  for(size_t i=0; i< messy_config_.size(); ++i)
  {
    messy_config_.at(i).header.stamp = ros::Time::now();// The time stamp _must_ be just before being published
    collision_object_pub_.publish( messy_config_.at(i) );
    ros::Duration(2.0).sleep();
    
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
    att_object.object.id = messy_config_.at(i).id;
    att_object.object.header.frame_id = "link_rhand_palm";
    att_object.object.header.stamp = ros::Time::now();
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
    
    att_collision_object_pub_.publish(att_object);
    ros::Duration(2.0).sleep();
    
    ROS_INFO("waitForService(SET_PLANNING_SCENE_DIFF_NAME)");
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

    if(!set_planning_scene_diff_client_.call(planning_scene_req_, planning_scene_res_)) {
      ROS_WARN("Can't get planning scene");
      return false;
    }
  
  }
  ROS_DEBUG("Objects should be back to the initial messy-spot");
  return true;
}

//! Get objects
/*!
  That is to the messy config.
  
  \param object id
  \param object_set Which object config should the function look for the object.
  \return an const_iterator to the object
*/
std::vector<arm_navigation_msgs::CollisionObject>::const_iterator
get_object(const std::string& object_id, const std::vector<arm_navigation_msgs::CollisionObject>& object_set)
{
  for(std::vector<arm_navigation_msgs::CollisionObject>::const_iterator i=object_set.begin(); i!=object_set.end(); ++i)
    if( !strcmp(object_id.c_str(), i->id.c_str()) )
      return i;
    
  return object_set.end();
}

//! Set the tidy_config_
/*!
  More...
  TODO should be from a file
*/
void
set_tidy_config()
{
  const double B_RADIUS = 0.065/2.;
  const double B_HEIGHT = 0.123;
  const double TABLE_THICKNESS = 0.050;
  
  //------------------------------------------------------------------CAN1
  arm_navigation_msgs::CollisionObject can1;
   
  can1.id = "CAN1"; 

  can1.header.seq = 1;
  can1.header.stamp = ros::Time::now();
  can1.header.frame_id = "/table";
  can1.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  arm_navigation_msgs::Shape can1_shape;
  
  can1_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can1_shape.dimensions.resize(2);
  can1_shape.dimensions[0] = B_RADIUS;
  can1_shape.dimensions[1] = B_HEIGHT;
    
  can1.shapes.push_back(can1_shape);

  geometry_msgs::Pose can1_tidy_pose;
  
  can1_tidy_pose.position.x = 0.; 
  can1_tidy_pose.position.y = 0.35;
  can1_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can1_tidy_pose.orientation.x = 0.;    
  can1_tidy_pose.orientation.y = 0.;    
  can1_tidy_pose.orientation.z = 0.;    
  can1_tidy_pose.orientation.w = 1.;    
      
  can1.poses.push_back(can1_tidy_pose);
  
  tidy_config_.push_back(can1);
  tidy_cfg_[can1.id] = can1;
  
  //------------------------------------------------------------------------------CAN2
  arm_navigation_msgs::CollisionObject can2;
   
  can2.id = "CAN2"; 

  can2.header.seq = 1;
  can2.header.stamp = ros::Time::now();
  can2.header.frame_id = "/table";
  can2.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can2_shape;
  
  can2_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can2_shape.dimensions.resize(2);
  can2_shape.dimensions[0] = B_RADIUS;
  can2_shape.dimensions[1] = B_HEIGHT;
    
  can2.shapes.push_back(can2_shape);

  geometry_msgs::Pose can2_tidy_pose;
  
  can2_tidy_pose.position.x = 0.; 
  can2_tidy_pose.position.y = 0.42;
  can2_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can2_tidy_pose.orientation.x = 0.;    
  can2_tidy_pose.orientation.y = 0.;    
  can2_tidy_pose.orientation.z = 0.;    
  can2_tidy_pose.orientation.w = 1.;    
      
  can2.poses.push_back(can2_tidy_pose);
  tidy_config_.push_back(can2);
  tidy_cfg_[can2.id] = can2;  
  //------------------------------------------------------------------------------CAN3
  arm_navigation_msgs::CollisionObject can3;
   
  can3.id = "CAN3"; 

  can3.header.seq = 1;
  can3.header.stamp = ros::Time::now();
  can3.header.frame_id = "/table";
  can3.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can3_shape;
  
  can3_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can3_shape.dimensions.resize(2);
  can3_shape.dimensions[0] = B_RADIUS;
  can3_shape.dimensions[1] = B_HEIGHT;
    
  can3.shapes.push_back(can2_shape);

  geometry_msgs::Pose can3_tidy_pose;
  
  can3_tidy_pose.position.x = -0.07; 
  can3_tidy_pose.position.y = 0.42;
  can3_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can3_tidy_pose.orientation.x = 0.;    
  can3_tidy_pose.orientation.y = 0.;    
  can3_tidy_pose.orientation.z = 0.;    
  can3_tidy_pose.orientation.w = 1.;    
      
  can3.poses.push_back(can3_tidy_pose);
  
  tidy_config_.push_back(can3);
  tidy_cfg_[can3.id] = can3;  
    //------------------------------------------------------------------------------CAN4
  arm_navigation_msgs::CollisionObject can4;
   
  can4.id = "CAN4"; 

  can4.header.seq = 1;
  can4.header.stamp = ros::Time::now();
  can4.header.frame_id = "/table";
  can4.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can4_shape;
  
  can4_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can4_shape.dimensions.resize(2);
  can4_shape.dimensions[0] = B_RADIUS;
  can4_shape.dimensions[1] = B_HEIGHT;
    
  can4.shapes.push_back(can4_shape);

  geometry_msgs::Pose can4_tidy_pose;
  
  can4_tidy_pose.position.x = -0.07; 
  can4_tidy_pose.position.y = 0.35;
  can4_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can4_tidy_pose.orientation.x = 0.;    
  can4_tidy_pose.orientation.y = 0.;    
  can4_tidy_pose.orientation.z = 0.;    
  can4_tidy_pose.orientation.w = 1.;    
      
  can4.poses.push_back(can4_tidy_pose);
  
  tidy_config_.push_back(can4);
  tidy_cfg_[can4.id] = can4;
}

void
set_hiro_home_joint_state()
{
  hiro_home_joint_state_.header.stamp = ros::Time::now();
  
  hiro_home_joint_state_.name.push_back("joint_chest_yaw");
  hiro_home_joint_state_.name.push_back("joint_rshoulder_yaw");
  hiro_home_joint_state_.name.push_back("joint_rshoulder_pitch");
  hiro_home_joint_state_.name.push_back("joint_relbow_pitch");
  hiro_home_joint_state_.name.push_back("joint_rwrist_yaw");
  hiro_home_joint_state_.name.push_back("joint_rwrist_pitch");
  hiro_home_joint_state_.name.push_back("joint_rwrist_roll");
   
  // For Hiro pose where his hands are in the back 
  hiro_home_joint_state_.position.push_back(0.);
  hiro_home_joint_state_.position.push_back(0.15707963267948966);  
  hiro_home_joint_state_.position.push_back(-2.7017696820872223);
  hiro_home_joint_state_.position.push_back(-2.6162485487395);
  hiro_home_joint_state_.position.push_back(2.0263272615654166);
  hiro_home_joint_state_.position.push_back(-0.38048177693476387);
  hiro_home_joint_state_.position.push_back(0.17453292519943295);
}

//! Go to a goal joint using move_arm pkg.
/*!
  Therefore, this includes both planning and execution.
  
  \param joint_state
  \param with_path_constraint
  \return whether successful
*/
bool
goto_goal_joint(const sensor_msgs::JointState& joint_state, bool with_path_constraint=false)
{
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_client("move_rarm",true);
  move_arm_client.waitForServer(); 
  ROS_INFO("Connected to the move_arm server");
  
  arm_navigation_msgs::MoveArmGoal goal;
  
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");  

  goal.motion_plan_request.group_name = "rarm";
  goal.motion_plan_request.num_planning_attempts = NUM_PLANNING_ATTEMPTS;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(ALLOWED_PLANNING_TIME);
  goal.motion_plan_request.planner_id= std::string("");
  
  goal.motion_plan_request.goal_constraints.joint_constraints.resize(joint_state.name.size());
  
  for(size_t i=0; i < goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_state.name.at(i);
    goal.motion_plan_request.goal_constraints.joint_constraints[i].position = joint_state.position.at(i);
    goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    goal.motion_plan_request.goal_constraints.joint_constraints[i].weight = 1.;
  }

  if (with_path_constraint)
  {
    goal.motion_plan_request.path_constraints.orientation_constraints.resize(1);  
    goal.motion_plan_request.path_constraints.orientation_constraints[0].header.frame_id = "link_base";
    goal.motion_plan_request.path_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
    goal.motion_plan_request.path_constraints.orientation_constraints[0].link_name = "link_rhand_palm";
    
    // Below is for the nominal orientation constraint that is rotated PI rad w.r.t y-axis 
    goal.motion_plan_request.path_constraints.orientation_constraints[0].orientation.x = 0.0;
    goal.motion_plan_request.path_constraints.orientation_constraints[0].orientation.y = 1.0;
    goal.motion_plan_request.path_constraints.orientation_constraints[0].orientation.z = 0.0;
    goal.motion_plan_request.path_constraints.orientation_constraints[0].orientation.w = 0.0;
    
    goal.motion_plan_request.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
//    goal.motion_plan_request.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::LINK_FRAME;
    goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = M_PI;//0.2;
    goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = M_PI/4;//0.2; 
    goal.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = M_PI;
  }
  
  move_arm_client.sendGoal(goal);
  ROS_INFO("Goal: SENT");
  
  bool finished_within_time = false;
  finished_within_time = move_arm_client.waitForResult(ros::Duration(2*60.0));// Should consider the speed of real joint movements

  if (!finished_within_time)
  {
    move_arm_client.cancelGoal();
    ROS_WARN("Timed out achieving goal A");
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm_client.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if(success)
    {
      ROS_DEBUG("Action finished: %s",state.toString().c_str());
      
      // Clear the path visualization
      trajectory_msgs::JointTrajectory empty_path;
      ros::Duration(0.5).sleep();// May be not necessary
      motion_plan_pub_.publish(empty_path);
        
      return true;
    }
    else
    {
      ROS_WARN("Action failed: %s",state.toString().c_str());
      return false;
    }
  } 
  return false;
}

bool
goto_home_pose()
{
  ROS_INFO("Initializing Hiro (offline-mode)");
  
  // TODO 2 modes
  sensor_msgs::JointState joint_state;
  
  joint_state.header.stamp = ros::Time::now();
  
  joint_state.name.push_back("joint_chest_yaw");
  joint_state.name.push_back("joint_rshoulder_yaw");
  joint_state.name.push_back("joint_rshoulder_pitch");
  joint_state.name.push_back("joint_relbow_pitch");
  joint_state.name.push_back("joint_rwrist_yaw");
  joint_state.name.push_back("joint_rwrist_pitch");
  joint_state.name.push_back("joint_rwrist_roll");
   
  // For Hiro pose where his hands are in the back 
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.15707963267948966);  
  joint_state.position.push_back(-2.7017696820872223);
  joint_state.position.push_back(-2.6162485487395);
  joint_state.position.push_back(2.0263272615654166);
  joint_state.position.push_back(-0.38048177693476387);
  joint_state.position.push_back(0.17453292519943295);
 
  return goto_goal_joint(joint_state);
}

bool 
goto_zero_pose()
{
  if( online_ )  
  {
    ROS_WARN("offline mode only!");
    return false; 
  }
 
  ROS_INFO("Testing collision with the table");
  
  // TODO 2 modes
  sensor_msgs::JointState joint_state;
  
  joint_state.header.stamp = ros::Time::now();
  
  joint_state.name.push_back("joint_chest_yaw");
  joint_state.name.push_back("joint_rshoulder_yaw");
  joint_state.name.push_back("joint_rshoulder_pitch");
  joint_state.name.push_back("joint_relbow_pitch");
  joint_state.name.push_back("joint_rwrist_yaw");
  joint_state.name.push_back("joint_rwrist_pitch");
  joint_state.name.push_back("joint_rwrist_roll");
   
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.);  
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.);
  joint_state.position.push_back(0.);
 
  return goto_goal_joint(joint_state);
}
};// end of: class PlannerManager

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_manager");
  ros::NodeHandle nh;
  
//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ROS_INFO("Waiting for /sense_see srv");
  ros::service::waitForService("/sense_see");
    
  ros::ServiceClient sense_see_client;
  sense_see_client = nh.serviceClient<hiro_sensor::Sense> ("/sense_see");
             
//  const size_t n = 100;
//  for(size_t i=0; i<n and ros::ok(); ++i)
//  {
////    pm.clear_messy_config();
//    
//    PlannerManager pm(nh);
//  
//    ros::Subscriber collision_object_sub;
//    collision_object_sub = nh.subscribe("collision_object", 10, &PlannerManager::collision_object_cb, &pm);
//  
//    ROS_INFO("Waiting for any object on the table.");
//    ros::Rate waiting_rate(30.);
//    while( !pm.is_messy_config_set() and ros::ok() )
//    {
//      //TODO why do we have to call this srv in the loop?
//      // It seems that this pm cannot get the collision obj igf it is called only once.
//      // At least, it takes 3 calls
//      // TODO make it efficient, e.g. a call to sense_see srv should return the collision objects immediately
//      
//      hiro_sensor::Sense::Request sense_see_req;
//      hiro_sensor::Sense::Response sense_see_res;
//      
//      sense_see_req.sensor_type = 1;
//      
//      if( !sense_see_client.call(sense_see_req, sense_see_res) ) 
//      {
//        ROS_WARN("Can't sense to see!");
//      }
//    
//      ros::spinOnce();
//      waiting_rate.sleep();
//    }
//   
//    pm.plan();
//    pm.extract();
//  }

  PlannerManager pm(nh);

  ros::Subscriber collision_object_sub;
  collision_object_sub = nh.subscribe("collision_object", 10, &PlannerManager::collision_object_cb, &pm);

  ROS_INFO("Waiting for sensing job to finish ...");// TODO make this is_finished checking elegant
  ros::Rate waiting_rate(30.);

  while( !pm.is_messy_config_set() and ros::ok() )
  {
    //TODO why do we have to call this srv in the loop?
    // It seems that this pm cannot get the collision obj igf it is called only once.
    // At least, it takes 3 calls
    // TODO make it efficient, e.g. a call to sense_see srv should return the collision objects immediately
    
    hiro_sensor::Sense::Request sense_see_req;
    hiro_sensor::Sense::Response sense_see_res;
    
    sense_see_req.sensor_type = 1;
    
    if( !sense_see_client.call(sense_see_req, sense_see_res) ) 
    {
      ROS_WARN("Can't sense to see!");
    }
  
    ros::spinOnce();
    waiting_rate.sleep();
  }
  ROS_INFO("sensing: finished");
  
  pm.plan();
  pm.collect();
//  pm.commit();

  ROS_INFO("Spinning...");
  ros::spin();
  
  return 0;
}
