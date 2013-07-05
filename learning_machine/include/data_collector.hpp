#ifndef DATA_COLLECTOR_HPP_INCLUDED
#define DATA_COLLECTOR_HPP_INCLUDED

#include "data_collector.hpp"
#include "data.hpp"
#include "utils.hpp"

#include <boost/graph/depth_first_search.hpp>

#include <Eigen/Dense>

namespace data_collector
{

struct DFSFoundGoalSignal {}; // exception for termination

template <class Graph>
class DataCollector: public boost::dfs_visitor<>
{
public:
DataCollector(std::string metadata_path,std::map<std::string, arm_navigation_msgs::CollisionObject> movable_obj_tidy_cfg)
{ 
  labels_ = data_util::get_labels(metadata_path);
  
  movable_obj_tidy_cfg = movable_obj_tidy_cfg;
}

//! Used for obtaining features (as input) _online_ during search.
bool
get_fval(const std::vector<typename boost::graph_traits<Graph>::edge_descriptor>& path,const Graph& g,Input* in)
{
  return get_fval_local<Graph>(path,g,in);
}

//!Used _online_ during search.
bool
get_samples(const std::vector<typename boost::graph_traits<Graph>::edge_descriptor>& path,const Graph& g,Data* samples)
{
  return get_samples_local<Graph>(path,g,samples);
}

private:
//! Extract a single sample from a single atom path
/*!
  More ...
  \param &path 
  \param &g
  \param *data
*/
template<typename LocalGraph>
bool
get_sample(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path,const LocalGraph& g,Data* data)
{
  Input in;
  if( !get_fval_local<LocalGraph>(path,g,&in) )
  {
    cerr << "get_fval() failed" << endl;
    return false;
  }

  Output out;
  if( !get_out<LocalGraph>(path,g,&out) )
  {
    cerr << "get_out() failed" << endl;
    return false;
  }
  
  data->insert( std::make_pair(in,out) );
  
  return true;
}

//! get_samples_from_hot_path()
/*!
  Concretely,
  consire the hot path: 
  ROOT --> A 
  then only one atom path: ROOT --> A 
  ROOT --> A --> B
  then there are 2 atom paths that can form new samples: ROOT--> A --> B and A --> B
*/
template<typename LocalGraph>
bool
get_samples_local(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path,const LocalGraph& g,Data* samples)
{
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
  {
    // get the subset of a hot_path_. see http://www.cplusplus.com/reference/vector/vector/vector/
    std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor> atom_path(i,path.end());
    
    if( !get_sample<LocalGraph>(atom_path,g,samples) )
      continue;
  }
  
  return true;
}

template<typename LocalGraph>
bool
get_out(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path, const LocalGraph& g,Output* out)
{
  *out = 0.;
  
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator j=path.begin(); j!=path.end(); ++j)
  {
    *out += get(edge_weight, g, *j);
  }
  
  // Check for bad out values
  if(*out == std::numeric_limits<double>::max())
  {
    cout << "[WARN] *out == std::numeric_limits<double>::max()" << endl;
    for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator j=path.begin(); j!=path.end(); ++j)
      cout << "e= " << get(edge_name, g, *j) << " has a weight of " << get(edge_weight, g, *j) << endl;
      
    return false;
  }
  
  // Scaling down, the dual (scale-up) is in class AstarHeuristics at file:astar_utils.hpp
  // TODO better to do this in prep_data()
  double scale = 0.1;
  *out *= scale;
  
  return true;
}

//! Extract input feature vector from a heuristics path
/*!
  (1)feature-input: 
  (1.1) symbolic features: position of an action Acti in the heu_path
  (1.2) geometric features: state descriptions, namely poses of movable objects, jstates
*/
template<typename LocalGraph>
bool
get_fval_local(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path, const LocalGraph& g,Input* in)
{
//  cerr << "in get_fval(), path: ";
//  for(typename std::vector<typename boost::graph_traits<Graph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
//    cerr << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
//  cerr << endl;
  
  RawInput r_in;

  // Geo. features extracted from the head vertex of this path: object pose+manipulability+jstates in the source vertex
  std::string srcstate;
  srcstate = get( edge_srcstate,g,path.at(0) );

  if( !get_geo_fval(srcstate,&r_in) )
    return false;
  
  // Sym. features
  if( !get_sym_fval<LocalGraph>(path,g,&r_in) )
    return false;
    
  // Convert the raw_in to in then return
//  cerr << "r_in.size()= " << r_in.size() << endl;
//    for(RawInput::iterator z=r_in.begin(); z!=r_in.end(); ++z)
//      cerr << z->first << "= " << z->second << endl;
//    cerr << "y= " << out << endl;
  return data_util::convert( r_in,in,labels_ );
}

//! Obtain symbolic features
/*!
  They are:
  (1) x^{s1}: position of actions
  (2) x^{s2}: length of the path
  (3) x^{s3}: transit transfer centric
  (4) x^{s4}: right left arm centric
*/
template<typename LocalGraph>
bool
get_sym_fval(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path, const LocalGraph& g,RawInput* r_in)
{
  // Extract x^{s1}: position of actions
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
  {
    std::string label;
    label = get(edge_name,g,*i);
    
    size_t idx;
    idx = i - path.begin() + 1;// Plus one because idx=0 is reserved for if Act_i is not in this path
    
    r_in->insert( std::make_pair(label,(double)idx) );
  }
  
  // Extract  x^{s2}: length of the path
  r_in->insert( std::make_pair("len",path.size()) );
  
  // Extract x^{s3}: transit transfer centric and x^{s4}: right left arm centric
  size_t n_transit = 0;
  size_t n_transfer = 0;
  size_t n_larm = 0;
  size_t n_rarm = 0;
  
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
  {
    std::string name = get(edge_name,g,*i);

    std::vector<std::string> name_parts;
    boost::split( name_parts,name,boost::is_any_of("_") );// e.g from "TRANSIT_RARM_HOME_MESSY-SPOT" to ...
    
    std::string op = name_parts.at(0);
    
    if( !strcmp(op.c_str(),"TRANSIT") )
      ++n_transit;
    else if( !strcmp(op.c_str(),"TRANSFER") )
      ++n_transfer;
      
    std::string rbt_id = name_parts.at(1);

    if( !strcmp(rbt_id.c_str(),"RARM") )
      ++n_rarm;
    else if( !strcmp(rbt_id.c_str(),"LARM") )
      ++n_larm;
  }
  
  r_in->insert( std::make_pair("TRANSIT-centric",(double)n_transit/(n_transit+n_transfer)) );
  r_in->insert( std::make_pair("TRANSFER-centric",(double)n_transfer/(n_transit+n_transfer)) );
  
  r_in->insert( std::make_pair("RARM-centric",(double)n_rarm/(n_rarm+n_larm)) );
  r_in->insert( std::make_pair("LARM-centric",(double)n_larm/(n_rarm+n_larm)) );
  
  return true;
}

//! Obtain geometric-feature values
/*!
  Geometric features are extracted from information of the head vertex
  (1) x^{g1} : robot joint states
  TODO (2) x^{g2} : manipulability measure m of robot state
  (3) x^{g3}: poses of movable objects
  (4) x^{g4}: shapes of movable objects (not used in this experiment July 5, 2013)
  (5) x^{g5}: Cartesian distances (of center of mass) of movable objects' current and final positions
*/
bool 
get_geo_fval(const std::string& srcstate,RawInput* r_in,const std::string& suffix="")
{
  // Init: parsing string of edge_srcstate
  std::vector<std::string> srcstate_parts;
  boost::split( srcstate_parts, srcstate, boost::is_any_of(";") );
  if( srcstate_parts.at(0).size() == 0 )
  {
    cerr << "srcstate_parts.at(0).size() == 0 -> get_geo_fval() returns false" << endl;
    return false;
  }

  std::map<std::string,double> jname_jpos_map;
  std::map< std::string,std::vector<double> > obj_pose_map;
  
  for(std::vector<std::string>::const_iterator i=srcstate_parts.begin(); i!=srcstate_parts.end(); ++i)
  {
    std::vector<std::string> comps;
    boost::split( comps, *i, boost::is_any_of(",") );
    
    if(comps.size()==8)// object's pose data: id, x, y, z, qx, qy, qz, qw
    {
      std::string id = comps.at(0);
      
      std::vector<double> pose(7);
      for(size_t j=1; j<8; ++j)// excluding obj_id, which is at(0)
      {
        pose.at(j-1) = boost::lexical_cast<double>( comps.at(j) );
      }
      
      obj_pose_map[id] = pose;
    }
    else if(comps.size()==2)// joint-name, joint-state
    {
      // Assume that no duplicated joint data 
      jname_jpos_map[comps.at(0)] = boost::lexical_cast<double>( comps.at(1) );
    }
    else
    {
      ROS_ERROR_STREAM("srcstate is corrupt; comps.size()= " << comps.size() );
      return false;
    }
  }

  // Extract x^{g1} : robot joint states; jname_jpos_map is set above
  for(std::map<std::string,double>::const_iterator i=jname_jpos_map.begin(); i!=jname_jpos_map.end(); ++i)
  {
    r_in->insert(  std::make_pair( i->first,i->second )  );
  }
  
  // Extract x^{g3}: poses of movable objects
  for(std::map< std::string,std::vector<double> >::const_iterator i=obj_pose_map.begin(); i!=obj_pose_map.end(); ++i)
  {
    std::string obj_id;
    obj_id = i->first;
      
    std::vector<std::string> labels(7);
    labels.at(0) = obj_id+".x"+suffix;
    labels.at(1) = obj_id+".y"+suffix;
    labels.at(2) = obj_id+".z"+suffix;
    labels.at(3) = obj_id+".qx"+suffix;
    labels.at(4) = obj_id+".qy"+suffix;
    labels.at(5) = obj_id+".qz"+suffix;
    labels.at(6) = obj_id+".qw"+suffix;
      
    for(size_t j=0; j<labels.size(); ++j)
    {
      std::string label;
      label = labels.at(j);
      
      double val;
      val = i->second.at(j);
      
      r_in->insert( std::make_pair(label,val) );
    }
  }
  
  // Extract x^{g5}: Cartesian distances (of center of mass) of movable objects' current and final positions
  for(std::map< std::string,std::vector<double> >::const_iterator i=obj_pose_map.begin(); i!=obj_pose_map.end(); ++i)
  {
    std::string label;
    label = std::string(i->first+".dist");
    
    Eigen::Vector3f curr_position;
    Eigen::Vector3f final_position;
    
    double val;// Euclidean distance
    val = sqrt( curr_position.dot(final_position) );
    
    r_in->insert( std::make_pair(label,val) );
  }
  
    
//  //(3) manipulability in the source vertex
//  ros::service::waitForService("get_manipulability");
//      
//  ros::ServiceClient gm_client;
//  gm_client = nh_.serviceClient<hiro_common::GetManipulability>("get_manipulability");
//  
//  hiro_common::GetManipulability::Request gm_req;
//  hiro_common::GetManipulability::Response gm_res;
//  
//  gm_req.jstate = jstate;
//  gm_req.jspace = "rarm_U_chest";// the biggest jspace of RARM
//  
//  if( !(gm_client.call(gm_req,gm_res)) )
//  {
//    ROS_DEBUG("GetManipulability srv call: failed");
//    return false;
//  }
//  r_in->insert( std::make_pair("RARM_manipulability"+suffix,gm_res.m) );
//  
//  gm_req.jspace = "larm_U_chest";// the biggest jspace of LARM
//  
//  if( !(gm_client.call(gm_req,gm_res)) )
//  {
//    ROS_DEBUG("GetManipulability srv call: failed");
//    return false;
//  }
//  r_in->insert( std::make_pair("LARM_manipulability"+suffix,gm_res.m) );
  
  return true;
}

std::vector<std::string> labels_;
utils::ObjCfg movable_obj_tidy_cfg;
};


template < typename GlobalGraph >
class BFSVisitor:public default_bfs_visitor 
{
public:
  BFSVisitor(vector< vector<typename graph_traits<GlobalGraph>::vertex_descriptor> >* predecessors_map)
  :predecessors_map_(predecessors_map)
  { }
  
  template < typename Edge, typename Graph >
  void examine_edge(Edge e, const Graph & g) const
  {
//    cout << "examine e=(" << get(vertex_name,g,source(e,g)) << "," << get(vertex_name,g,target(e,g)) << ")" << endl;
    predecessors_map_->at(target(e,g)).push_back(source(e,g));
  }

private:
vector< vector<typename graph_traits<GlobalGraph>::vertex_descriptor> >* predecessors_map_;
};

template<typename Graph>
void
backtrack(const Graph& g
         ,const std::vector< std::vector<typename graph_traits<Graph>::vertex_descriptor> >& predecessors_map
         ,const typename graph_traits<Graph>::vertex_descriptor& v
         ,std::vector< std::vector<typename graph_traits<Graph>::edge_descriptor> >* paths
         )
{
//  cout << "v= " << get(vertex_name,g,v) << endl;
//  
//  cout << "paths.size()= " << paths->size() << endl;
//  for(size_t i=0; i<paths->size(); ++i)
//  {
//    cout << "path th= " << i << ": " << endl;
//    for(size_t j=0; j<paths->at(i).size(); ++j)
//    {
//      typename graph_traits<Graph>::edge_descriptor e = paths->at(i).at(j);
//      cout << "e(" << get(vertex_name,g,source(e,g)) << "," << get(vertex_name,g,target(e,g)) << "),";
//    }
//    cout << endl;
//  }
  
  if( predecessors_map.at(v).empty() )
  {
    return;
  }
  
  size_t n_predecessors;
  n_predecessors = predecessors_map.at(v).size();
  
  // find paths in path_set whose last edge has the source v
  // Assume that all matched_paths are unique
  std::vector< std::vector<typename graph_traits<Graph>::edge_descriptor> > matched_paths;

  for(size_t j=0; j<paths->size(); ++j)
  {
    typename graph_traits<Graph>::edge_descriptor last_e;
    last_e = paths->at(j).back();
    
    if(source(last_e,g) == v)
    {
      matched_paths.push_back( paths->at(j) );
    }
  }
    
  // Duplicate matched_paths as many as (n_predecessors-1)
//  cout << "matched_paths.size()= " << matched_paths.size() << endl;
  for(size_t i=0; i<matched_paths.size(); ++i)
  {
    for(size_t j=0; j<(n_predecessors-1); ++j)
    {
      paths->push_back(matched_paths.at(i));
    }
  }

  for(size_t i=0; i< predecessors_map.at(v).size(); ++i)
  {
    typename graph_traits<Graph>::vertex_descriptor p;
    p = predecessors_map.at(v).at(i);
    
    typename graph_traits<Graph>::edge_descriptor e;
    e = boost::edge(p,v,g).first;
    
    // find _one_ (first found) path in path_set whose last edge has the source v,
    // then add e to that path
    // TODO is it possible that there exist multiple unique path that end with (v,any) ?
    bool found = false;
    for(size_t j=0; j<paths->size(); ++j)
    {
      typename graph_traits<Graph>::edge_descriptor last_e;
      last_e = paths->at(j).back();
      
      if(source(last_e,g) == v)
      {
        paths->at(j).push_back(e);
        found = true;
        break;
      }
    }

    if(!found)
    {
      // Create a new path with e, then push to paths
      std::vector<typename graph_traits<Graph>::edge_descriptor> path;
      path.push_back(e);
      
      paths->push_back(path);
    }
    
    backtrack<Graph>(g,predecessors_map,p,paths);
  }// for each predecessor
}

}// namespace data_collector
#endif // #ifndef DATA_COLLECTOR_HPP_INCLUDED
