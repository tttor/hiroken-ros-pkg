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
#include "ml_util.hpp"

using namespace std;
using namespace boost;

struct FoundGoalSignal {}; // exception for termination

template <typename GPMGraph,typename LearningMachine>
class AstarVisitor: public boost::default_astar_visitor
{
public:
AstarVisitor(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,std::vector< std::vector<double> >* ml_data,size_t ml_mode)
: goal_(goal), gpm_(gpm), learner_(learner), ml_data_(ml_data), ml_mode_(ml_mode)
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

  // Obtain samples from paths from (root,root+1, ..., v) to adjacent of v where the edge costs are already defined
  Data samples;
  gpm_->get_samples_online(v,&samples);
//    cerr << "online training: samples.size()= " << samples.size() << endl;

  // Utilize samples
  if(ml_mode_==ml_util::NO_ML or ml_mode_==ml_util::SVR_OFFLINE)// Store samples for offline training
  {
    // Write samples to a libsvmdata format
    std::string libsvmdata_path;
    libsvmdata_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/tr_data.libsvmdata";// _must_ be synch with the one in planner_manager.cpp
    
    std::string delta_libsvmdata_path;
    delta_libsvmdata_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/delta_tr_data.libsvmdata";// _must_ be synch with the one in planner_manager.cpp
    
    write_libsvm_data(samples,libsvmdata_path,std::ios::app);
    write_libsvm_data(samples,delta_libsvmdata_path,std::ios::app);
    
    // Write samples to a CSV format
    std::string csv_path;
    csv_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/tr_data.csv";// _must_ be synch with the one in planner_manager.cpp

    std::string delta_csv_path;
    delta_csv_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/delta_tr_data.csv";// _must_ be synch with the one in planner_manager.cpp
    
    std::ofstream csv;
    csv.open( csv_path.c_str(),std::ios::app );
    
    std::ofstream delta_csv;
    delta_csv.open( delta_csv_path.c_str(),std::ios::app );
    
    for(Data::const_iterator i=samples.begin(); i!=samples.end(); ++i)
    {
      for(Input::const_iterator j=i->first.begin(); j!=i->first.end(); ++j)
      {
        csv << *j << ",";// Write input=feature values
        delta_csv << *j << ",";
      }
        
      csv << i->second << std::endl;
      delta_csv << i->second << std::endl;
    }
    csv.close();
    delta_csv.close();
  }
  else if(ml_mode_==ml_util::LWPR_ONLINE)// Train online during search (the online LWPR)
  {
    for(Data::const_iterator i=samples.begin(); i!=samples.end(); ++i)
    {
      std::vector<double> x;
      x = i->first;// input
      
      std::vector<double> y(1);
      y.at(0) = i->second;// observed output

      std::vector<double> y_fit_test;// for prediction before the model is updated with this samples
      y_fit_test = learner_->predict(x);

      std::vector<double> y_fit;// for prediction on training data
      y_fit = learner_->update( x,y );// likely that this prediction after the model is updated, it differs from the prediction before updating 
//      cerr << "y_fit= " << y_fit.at(0) << endl;
      
      if( y_fit.empty() )
      {
        std::cerr << "update(x,y)= NOT OK" << std::endl;
        continue;// with the next samples
      }
      
      // Keep ml-related data 
      std::vector<double> ml_datum;
      ml_datum.push_back(y_fit_test.at(0));// 0 
      ml_datum.push_back(y_fit.at(0));// 1
      ml_datum.push_back(y.at(0));// 2
      ml_datum.push_back(learner_->nData());// 3
      
      ml_data_->push_back(ml_datum);
    }
    
    // Store the updated model
    std::string lwpr_model_path;
    lwpr_model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/lwpr.bin";
    
    if( !learner_->writeBinary(lwpr_model_path.c_str()) )
     std::cerr << "learner_->writeBinary()= NOT OK" << std::endl;
  }
  
  // Set its color to black=examined
  gpm_->mark_vertex(v);
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
LearningMachine* learner_;  
std::vector< std::vector<double> >* ml_data_;
size_t ml_mode_;

};

template <typename GPMGraph, typename CostType, typename LearningMachine>
class AstarHeuristics: public astar_heuristic<GPMGraph, CostType>
{
public:
typedef typename graph_traits<GPMGraph>::vertex_descriptor Vertex;

AstarHeuristics(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,size_t ml_mode)
: goal_(goal), gpm_(gpm), learner_(learner), ml_mode_(ml_mode)
{ }

CostType 
operator()(Vertex v)
{
  cerr << "Compute h(" << v << "): BEGIN" << endl;
  double h;
  
  if( (ml_mode_==ml_util::NO_ML)or(v==goal_) )// or ml_mode=UCS, no learning
  {
      h = 0.;
  }
  else// get the heuristic from a learning machine, so far either ml_mode= ml_util::SVR_OFFLINE or ml_mode=ml_util::LWPR_ONLINE
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
size_t ml_mode_;
};

#endif // #ifndef ASTAR_UTILS_HPP_INCLUDED
