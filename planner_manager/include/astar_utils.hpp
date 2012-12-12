#ifndef ASTAR_UTILS_HPP_INCLUDED
#define ASTAR_UTILS_HPP_INCLUDED

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/property_iter_range.hpp>

#include "geo_planner_manager.hpp"

// NOTE that all Vertex, Edge template MUST be TMMVErtex and EdgeVertex, because this class is contructed with an instance of GeometricPlannerManager, which is a friend of PlannerManager who is the owner of tmm_, whose vertex and edge are of TMMVertex and TMMEdge.

using namespace std;
using namespace boost;

struct FoundGoalSignal {}; // exception for termination

template <typename GPMGraph>
class AstarVisitor: public boost::default_astar_visitor
{
public:
  AstarVisitor(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm)
  : goal_(goal), gpm_(gpm)
  { }
  
  template <typename Graph>
  void 
  examine_vertex(typename Graph::vertex_descriptor v, Graph& g) 
  {
    cerr << "Examine v= " << get(vertex_name,g,v) << endl;
    
    gpm_->mark_vertex(v);
    
    if(v == goal_)
      throw FoundGoalSignal();
    
    // Do geometric planning for each out-edge of this vertex v
    typename graph_traits<Graph>::out_edge_iterator oei, oei_end;
    for(tie(oei,oei_end) = out_edges(v, g); oei!=oei_end; ++oei)
    {
      gpm_->plan(*oei);
    }
    
    // Update jstates of adjacent vertex of this vertex v
    gpm_->set_av_jstates(v);
  }
  
  template <typename Graph>
  void 
  initialize_vertex(typename Graph::vertex_descriptor v, Graph& g) 
  {
    gpm_->init_vertex(v);
  }
  
private:
  typename GPMGraph::vertex_descriptor goal_;
  GeometricPlannerManager* gpm_;
};

template <typename Graph, typename CostType>
class AstarHeuristics: public astar_heuristic<Graph, CostType>
{
public:
typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

AstarHeuristics(Vertex goal)
: goal_(goal)
{ }

CostType 
operator()(Vertex u)
{
  if(u == goal_)
    return 0.;
  
  double h;
  h = 0.;
  
  return h;
}

private:
Vertex goal_;
};

#endif // #ifndef ASTAR_UTILS_HPP_INCLUDED
