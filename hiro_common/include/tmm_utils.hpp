#ifndef TMM_UTILS_HPP_INCLUDED
#define TMM_UTILS_HPP_INCLUDED

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/CollisionObject.h>

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/property_iter_range.hpp>
#include <boost/graph/adjacency_list_io.hpp>
#include <boost/graph/property_iter_range.hpp>
#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
#error adjacency_list_io.hpp has not been ported to work with VC++
#endif

using namespace boost;
using namespace std;

enum edge_jspace_t { edge_jspace };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, jspace);
}

enum edge_plan_t { edge_plan };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, plan);
}

enum edge_planstr_t { edge_planstr };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, planstr);
}

enum edge_srcstate_t { edge_srcstate };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(edge, srcstate);
}

enum vertex_jstates_t { vertex_jstates };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(vertex, jstates);
}

enum vertex_wstate_t { vertex_wstate };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(vertex, wstate);
}

enum vertex_heu_t { vertex_heu };
namespace boost 
{
  BOOST_INSTALL_PROPERTY(vertex, heu);
}

typedef boost::property< edge_name_t, std::string, boost::property<edge_weight_t, double> > TGEdgeProperty;
typedef boost::property< edge_name_t, std::string, boost::property<edge_weight_t, double, boost::property<edge_jspace_t, std::string, boost::property<edge_plan_t, trajectory_msgs::JointTrajectory, property<edge_color_t, std::string, property<edge_srcstate_t, std::string, property<edge_planstr_t, std::string> > > > > > > TMMEdgeProperty;
    
typedef boost::property<vertex_name_t, std::string> TGVertexProperty;
typedef boost::property<vertex_name_t, std::string, boost::property< vertex_jstates_t, sensor_msgs::JointState, property<vertex_color_t, default_color_type, property<vertex_wstate_t,std::vector<arm_navigation_msgs::CollisionObject>, property<vertex_heu_t, double> > > > > TMMVertexProperty;
    
typedef boost::adjacency_list<setS, vecS, directedS, TGVertexProperty, TGEdgeProperty > TaskGraph;// for task graph, not allowing parallel edges.
typedef boost::adjacency_list<vecS, vecS, directedS, TMMVertexProperty, TMMEdgeProperty > TaskMotionMultigraph;// for task motion multigraph, allowing parallel edges.
    
typedef TaskGraph::vertex_descriptor TGVertex;
typedef TaskMotionMultigraph::vertex_descriptor TMMVertex;

typedef TaskGraph::edge_descriptor TGEdge;
typedef TaskMotionMultigraph::edge_descriptor TMMEdge;

typedef property_map<TaskGraph, vertex_name_t>::type TGVertexNameMap;

typedef property_map<TaskGraph, edge_name_t>::type TGEdgeNameMap;
typedef property_map<TaskGraph, edge_weight_t>::type TGEdgeWeightMap;

typedef property_map<TaskMotionMultigraph, vertex_name_t>::type TMMVertexNameMap;
typedef property_map<TaskMotionMultigraph, vertex_color_t>::type TMMVertexColorMap;

typedef property_map<TaskMotionMultigraph, edge_name_t>::type TMMEdgeNameMap;
typedef property_map<TaskMotionMultigraph, edge_weight_t>::type TMMEdgeWeightMap;
typedef property_map<TaskMotionMultigraph, edge_jspace_t>::type TMMEdgeJspaceMap;
typedef property_map<TaskMotionMultigraph, edge_color_t>::type TMMEdgeColorMap;

template<typename NameMap>
class VertexPropWriter_1
{
public:
  VertexPropWriter_1(NameMap name_map)
    : name_map_(name_map)
  {}
  
  template <typename Vertex>
  void operator()(ostream& out, const Vertex& v) const
  {
    out << "["
        << "label=\"" << name_map_[v] << "\""
        << ",fontsize=\"10\"" 
        << "]";
  }
private:
  NameMap name_map_;
};

template <typename NameMap, typename WeightMap>
class EdgePropWriter_1  
{
public:
  EdgePropWriter_1 (NameMap name_map, WeightMap weight_map) 
    : name_map_(name_map)
    , weight_map_(weight_map)
  {}
  
  template <typename Edge>
  void 
  operator()(ostream &out, const Edge& e) const 
  {
    out << "["
    << "label=\"" << name_map_[e] 
    << "\\" << "n" << weight_map_[e] 
    << "\""
    << ",fontsize=\"10\""
    << "]";
  }
private:
  NameMap name_map_;  
  WeightMap weight_map_;
};

template <typename NameMap, typename WeightMap, typename JspaceMap, typename ColorMap>
class TMMEdgePropWriter  
{
public:
  TMMEdgePropWriter(NameMap name_map, WeightMap weight_map, JspaceMap jspace_map, ColorMap color_map) 
    : name_map_(name_map)
    , weight_map_(weight_map)
    , jspace_map_(jspace_map)
    , color_map_(color_map)
  {}
  
  template <typename Edge>
  void 
  operator()(ostream &out, const Edge& e) const 
  {
    out << "["
    << "label=\"" << name_map_[e] 
    << "\\" << "n" << jspace_map_[e] 
    << "\\" << "n" << weight_map_[e] 
    << "\",fontsize=\"10\""
    << ",color=\"" << color_map_[e] << "\""
    << "]";
  }
private:
  NameMap name_map_;  
  WeightMap weight_map_;
  JspaceMap jspace_map_;
  ColorMap color_map_;
};

template<typename NameMap, typename ColorMap>
class TMMVertexPropWriter
{
public:
  TMMVertexPropWriter(NameMap n, ColorMap c)
  : name_map_(n), color_map_(c)
  {}
  
  template <typename Vertex>
  void operator()(ostream& out, const Vertex& v) const
  {
    std::string color_str = "black";
    if(color_map_[v]==color_traits<boost::default_color_type>::gray())// Prioritize solution nodes
    {
      color_str = "blue";// belongs to the solution path
    }
    else if(color_map_[v]==color_traits<boost::default_color_type>::black())
    {
      color_str = "red";// for examined vertex
    }

    out << "["
    << "label=\"" << name_map_[v] << "\",fontsize=\"10\"" 
    << ",color=\"" << color_str << "\""
    << "]";
  }
private:
  NameMap name_map_;
  ColorMap color_map_;
};

template <typename ColorMap>
class PlannedEdgeFilter
{
public:
  PlannedEdgeFilter()
  { }
  
  PlannedEdgeFilter(ColorMap color_map)
    : color_map_(color_map) 
  { }
  
  template <typename Edge>
  bool 
  operator()(const Edge& e) const 
  {
    return ( (!strcmp(color_map_[e].c_str(),"green"))or(!strcmp(color_map_[e].c_str(),"red"))or(!strcmp(color_map_[e].c_str(),"blue")) );
  }

private:
  ColorMap color_map_;  
};

template <typename ColorMap>
class SolEdgeFilter
{
public:
  SolEdgeFilter()
  { }
  
  SolEdgeFilter(ColorMap color_map)
    : color_map_(color_map) 
  { }
  
  template <typename Edge>
  bool 
  operator()(const Edge& e) const 
  {
    return (!strcmp(color_map_[e].c_str(),"blue"));
  }

private:
  ColorMap color_map_;  
};

#endif // #ifndef TMM_UTILS_HPP_INCLUDED
