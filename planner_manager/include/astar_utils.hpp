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
#include "hiro_utils.hpp"

using namespace std;
using namespace boost;

struct FoundGoalSignal {}; // exception for termination


template <typename GPMGraph,typename LearningMachine>
class AstarVisitor: public boost::default_astar_visitor
{
public:
AstarVisitor(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,std::vector< std::vector<double> >* ml_data,size_t ml_mode,std::string ml_hot_path,double* total_gp_time,size_t* n_exp_op)
: goal_(goal), gpm_(gpm), learner_(learner), ml_data_(ml_data), ml_mode_(ml_mode), ml_hot_path_(ml_hot_path), total_gp_time_(total_gp_time), n_exp_op_(n_exp_op)
{ }

template <typename Graph>
void 
examine_vertex(typename Graph::vertex_descriptor v, Graph& g) 
{
  cerr << "Examine v= " << get(vertex_name,g,v) << endl;
  ++(*n_exp_op_);// Increment the number of expansion operations
  gpm_->mark_vertex(v);// Set its color to black=examined
  
  if(v == goal_)
    throw FoundGoalSignal();

  // Do geometric planning (grasp planning at the beginning then motion planning) for one motion plan for each unique high level action connecting v to its adjacent vertex
  // The rule is do geo. planning starting from the lowest JSPACE then stop if one motion plan is found
  // This is because lower-dimensional motion planning is likely (not always) to have lower cost; to speed up the planning as well
  // For that, we have to iterate over all adjacent vertices, then sort edges of v->av
  // In this step,we also remove edges that does not have valid un/grasp pose as goal pose for motion planning
  size_t n_avi = 0;// For multigraph, n_avi = n_out-edges. Note this make an excessive call to gpm_->plan as we iterate over avi
  
  typename graph_traits<Graph>::adjacency_iterator avi, avi_end;
  for(tie(avi,avi_end)=adjacent_vertices(v,g); avi!=avi_end; ++avi)  
  {
    ++n_avi;
      
    // Identify edges that connect v and its adjacent 
    // Note: because this is multigraph, we can not simply use boost::edge(u,v,g).first
    std::vector<typename Graph::edge_descriptor> conn_edges;// conn_ for connecting
    
    typename graph_traits<Graph>::out_edge_iterator oei, oei_end;
    for(tie(oei,oei_end) = out_edges(v,g); oei!=oei_end; ++oei)
    {
      typename Graph::vertex_descriptor t;
      t = target(*oei,g);
      
      if(t == *avi)
        conn_edges.push_back(*oei);
    }
    
    // Sort edges in conn_edges based on |Jspace|
//    // This if we can use lambda, enable lambda: put this somewhere in CMakeList.txt SET(CMAKE_CXX_FLAGS "-std=c++0x") # Add c++11 functionalit
//    std::sort(conn_edges.begin(),conn_edges.end(),
//              [&conn_edges](std::pair<typename Graph::edge_descriptor,size_t> p1,std::pair<typename Graph::edge_descriptor,size_t> p2) {return p1.second < p2.second;});
    // This is just a workaround, work only for 2 jspace 
    if(conn_edges.size() == 2)
    {
      if(get_jspace_size(get(edge_jspace,g,conn_edges.at(0))) > get_jspace_size(get(edge_jspace,g,conn_edges.at(1))))
      {
        // Swap
        typename Graph::edge_descriptor tmp_e;
        tmp_e = conn_edges.at(0);
        
        conn_edges.at(0) = conn_edges.at(1);
        conn_edges.at(1) = tmp_e;
      }
    }
    
//    cout << "Sorted edges connecting to " << get(vertex_name,g,*avi) << " -> " << n_avi << "-th" << endl;
//    for(size_t i=0; i<conn_edges.size(); ++i)
//    {
//      cout << i << " -> " << get(edge_jspace,g,conn_edges.at(i)) << endl;
//    }
    
    // Do geometric planning
    for(size_t i=0; i<conn_edges.size(); ++i)
    {
      typename Graph::edge_descriptor e;
      e = conn_edges.at(i);

      double gp_time = 0.;
      bool found_mp = false;
      bool found_gp = false;// gp for at least one goal pose
      
      gpm_->plan(e,&gp_time,&found_gp,&found_mp);
      *total_gp_time_ += gp_time;
      
      if(!found_gp)
      {
        gpm_->remove_ungraspable_edge(e);
        continue;
      }
      
      if(found_mp) break;
    }// for all conn_edges
  }// for all avi
  cout << get(vertex_name,g,v) << " has n_avi= " << n_avi << endl;
  
  // Update jstates of adjacent vertex av of this vertex v to the cheapest existing-just-planned edge
  gpm_->set_av_jstates(v); 

  // Obtain samples from paths from (root,root+1, ..., v) to adjacent of v where the edge costs are already defined
  Data samples;
  if(ml_mode_ != ml_util::NO_ML)
    gpm_->get_samples_online(v,&samples);

  // Utilize samples
  if((ml_mode_ == ml_util::SVR_OFFLINE)or(ml_mode_ == ml_util::NO_ML_BUT_COLLECTING_SAMPLES))// Store samples for offline (interleaved) training and for offline tuning
  {
    // Write samples + the delta to a libsvmdata format: APPENDING
    std::string libsvmdata_path = std::string(ml_hot_path_+"/tr_data.libsvmdata");;
    std::string delta_libsvmdata_path = std::string(ml_hot_path_+"/delta_tr_data.libsvmdata");
    
    data_util::write_libsvm_data(samples,libsvmdata_path,std::ios::app);
    data_util::write_libsvm_data(samples,delta_libsvmdata_path,std::ios::app);
    
    // Write samples + the delta to a CSV format: APPENDING
    std::string csv_path = std::string(ml_hot_path_+"/tr_data.csv");
    std::string delta_csv_path = std::string(ml_hot_path_+"/delta_tr_data.csv");
    
    data_util::write_csv_data(samples,csv_path,std::ios::app);
    data_util::write_csv_data(samples,delta_csv_path,std::ios::app);
  }
  else if(ml_mode_==ml_util::LWPR_ONLINE)// Train online during search (the online LWPR)
  {
    cerr << "LWPR_ONLINE: training with " << samples.size() << " samples: BEGIN" << endl;
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
    cerr << "LWPR_ONLINE: training with " << samples.size() << " samples: END" << endl;
  }
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
std::string ml_hot_path_;
double* total_gp_time_;
size_t* n_exp_op_;
};

template <typename GPMGraph, typename CostType, typename LearningMachine>
class AstarHeuristics: public astar_heuristic<GPMGraph, CostType>
{
public:
typedef typename graph_traits<GPMGraph>::vertex_descriptor Vertex;

AstarHeuristics(typename GPMGraph::vertex_descriptor goal, GeometricPlannerManager* gpm,LearningMachine* learner,size_t ml_mode,ml_util::PrepData prep_data)
: goal_(goal), gpm_(gpm), learner_(learner), ml_mode_(ml_mode), prep_data_(prep_data)
{ }

CostType 
operator()(Vertex v)
{
  cerr << "Compute h(" << v << "): BEGIN" << endl;
  double h;
  
  if(v == goal_)
  {
    cerr << "(v==goal_) -> h = 0." << endl;
    h = 0.;
  }
  else if((ml_mode_ == ml_util::NO_ML)or(ml_mode_ == ml_util::NO_ML_BUT_COLLECTING_SAMPLES))
  {
    cerr << "(ml_mode_==NO_ML or NO_ML_BUT_COLLECTING_SAMPLES) -> h = 0." << endl;
    h = 0.;
  }
  else if( (ml_mode_ == ml_util::SVR_OFFLINE) or (ml_mode_ == ml_util::LWPR_ONLINE) )
  {
    // Extract feature x
    Input x;
    if( gpm_->get_fval(v,&x) )
    {
      // Preprocess data if necessary
      bool has_to_prep_data = false;
      bool has_to_postp_data = false;
      
      if(ml_mode_ == ml_util::SVR_OFFLINE)
      {
        has_to_prep_data = true;
        has_to_postp_data = true;
      }
        
      if(has_to_prep_data)
      {
        // All below should mimic the proprocess_data() routine implemented in matlab
        double* x_ptr = &x[0];
        Eigen::Map<Eigen::VectorXd> tmp_x(x_ptr,x.size());
        
        // Centering; PCA needs centered_x
        Eigen::VectorXd centered_x;
        centered_x = tmp_x - prep_data_.mu_X;
        
        // Project to new space
        Eigen::VectorXd new_x;
        new_x = centered_x.transpose() * prep_data_.T;
        
        // Reduce dim
        Eigen::VectorXd lodim_x;
        lodim_x = new_x.head(prep_data_.lo_dim);
        
        // Convert back to std::vector
        x.clear();
        x.resize(lodim_x.size());
        for(size_t i=0; i<lodim_x.size(); ++i)
          x.at(i) = lodim_x(i);
      }
        
      // Predict yp with the learning machine
      std::vector<double> yp;
      yp = learner_->predict(x);

      // Assign
      if( !yp.empty() )
      {
        if(has_to_postp_data)
        {
          yp.at(0) += prep_data_.mu_y(0);
        }
        
        // Scaling, its dual (scale-down) is in get_out() at file:data_collector.hpp
        // TODO better if done together in postp_data()
        const double scale_up = 10.;
        h = yp.at(0) * scale_up;
      }
      else
      {
        cerr << "yp.empty() -> learner_->predict(x) FAILED -> h = 0." << endl;
        h = 0.;
      }
    }
    else
    {
      cerr << "gpm_->get_fval(v,&x): FAILED -> h = 0." << endl;
      h = 0.;
    }
  }
  
  // Put in the main tmm!
  if(h < 0.)// negative value
  {
    cerr << "h= " << h << " -> h = 0." << endl;
    h = 0.;
  } 
  
  cerr << "h(" << v << ")= " << h << endl;
  gpm_->put_heu(v,h);
  
  cerr << "Compute h(" << v << "): END" << endl;
  return h;
}

private:
Vertex goal_;
GeometricPlannerManager* gpm_;
LearningMachine* learner_;
size_t ml_mode_;
ml_util::PrepData prep_data_;
};

#endif // #ifndef ASTAR_UTILS_HPP_INCLUDED
