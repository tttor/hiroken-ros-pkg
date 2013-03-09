#ifndef ASTAR_UTILS_HPP_INCLUDED
#define ASTAR_UTILS_HPP_INCLUDED

#include "astar_utils.hpp"

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/property_iter_range.hpp>

#include <lwpr.hh>

#include "geo_planner_manager.hpp"
#include "data.hpp"

using namespace std;
using namespace boost;

struct FoundGoalSignal {}; // exception for termination

template <typename GPMGraph,typename LearningMachine>
class AstarVisitor: public boost::default_astar_visitor
{
public:
AstarVisitor(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,size_t mode)
: goal_(goal), gpm_(gpm), learner_(learner), mode_(mode)
{ }

template <typename Graph>
void 
examine_vertex(typename Graph::vertex_descriptor v, Graph& g) 
{
  cerr << "Examine v= " << get(vertex_name,g,v) << endl;
  
  if(v == goal_)
    throw FoundGoalSignal();

  // Do geometric planning (grasp and motion planning) for each out-edge of this vertex v
  std::vector<typename Graph::edge_descriptor> ungraspable_edges;// invalid because no grasp/ungrasp pose as the goal pose for the motion planning
  
  typename graph_traits<Graph>::out_edge_iterator oei, oei_end;
  for(tie(oei,oei_end) = out_edges(v, g); oei!=oei_end; ++oei)
  {
    bool success = false;
    success = gpm_->plan(*oei);
    
    if(!success)
      ungraspable_edges.push_back(*oei);
  }
  
  for(typename std::vector<typename Graph::edge_descriptor>::iterator i=ungraspable_edges.begin(); i!=ungraspable_edges.end(); ++i)
    gpm_->remove_ungraspable_edge(*i);
  
  // Update jstates of adjacent vertex av of this vertex v to the cheapest existing-just-planned edge
  gpm_->set_av_jstates(v); 

  // Train online
  switch(mode_)
  {
    case 1:
    {
      break;
    }
    case 2:
    {
      break;
    }
    case 3:// The learning machine is the LWPR
    {
      // Train online during search
      Data samples;

      gpm_->get_samples_online(v,&samples);// get samples from paths from (root,root+1, ..., v) to adjacent of v
      cerr << "online: samples.size()= " << samples.size() << endl;
      
      for(Data::const_iterator i=samples.begin(); i!=samples.end(); ++i)
      {
        doubleVec x;
        x = i->first;
        
        doubleVec y(1);
        y.at(0) = i->second;

        doubleVec yp;
        yp = learner_->update( x,y );
        
        if( yp.empty() )
           std::cerr << "update(x,y)= NOT OK" << std::endl;
      }
      
      // Store the updated model
      std::string lwpr_model_path;
      lwpr_model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/lwpr.bin";
      
      if( !learner_->writeBinary(lwpr_model_path.c_str()) )
       std::cerr << "learner_->writeBinary()= NOT OK" << std::endl;

      break;
    }
  }

  // Set its color to black=examined
  gpm_->mark_vertex(v);
}
  
template <typename Graph>
void 
initialize_vertex(typename Graph::vertex_descriptor v, Graph& g) 
{
  if(mode_==1)
    gpm_->init_vertex(v);
}
  
private:
typename GPMGraph::vertex_descriptor goal_;
GeometricPlannerManager* gpm_;
LearningMachine* learner_;  
size_t mode_;
};

template <typename GPMGraph, typename CostType, typename LearningMachine>
class AstarHeuristics: public astar_heuristic<GPMGraph, CostType>
{
public:
typedef typename graph_traits<GPMGraph>::vertex_descriptor Vertex;

AstarHeuristics(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,size_t mode)
: goal_(goal), gpm_(gpm), learner_(learner), mode_(mode)
{ }

CostType 
operator()(Vertex v)
{
  cerr << "Compute h(" << v << "): BEGIN" << endl;
  double h;
  
  if( (mode_==1)or(v==goal_) )// or mode=UCS, no learning
  {
      h = 0.;
  }
  else// get the heuristic from a learning machine
  {
    // Extract feature x
    Input x;
    if ( !gpm_->get_fval(v,&x) )
    {
      h = 0.;
    }
    else
    {
      // Predict yp
      std::vector<double> yp;
      yp = learner_->predict(x);
      
      // Assign
      const double scale_up = 10.;
      if( !yp.empty() )
        h = yp[0] * scale_up;
      else
        h = 0.;
    }
  }

  gpm_->put_heu(v,h);
  
  cerr << "Compute h(" << v << "): END" << endl;
  return h;
}

private:
Vertex goal_;
GeometricPlannerManager* gpm_;
LearningMachine* learner_;
size_t mode_;
};

#endif // #ifndef ASTAR_UTILS_HPP_INCLUDED
