#ifndef DATA_COLLECTOR_HPP_INCLUDED
#define DATA_COLLECTOR_HPP_INCLUDED

#include "data_collector.hpp"
#include "data.hpp"

#include <boost/graph/depth_first_search.hpp>

struct DataCollectorFoundGoalSignal {}; // exception for termination

template <class Graph>
class DataCollector: public boost::dfs_visitor<>
{
public:
DataCollector(Data* data,std::string metadata_path)
: data_(data), in_(0)
{ 
  labels_ = get_labels(metadata_path);
}

//! This contructor is used to get features as inputs during search, used to supply the learning machine in order to output the heuristic
DataCollector()
: data_(0)
{ }

template <class DfsGraph>
void 
discover_vertex(typename boost::graph_traits<DfsGraph>::vertex_descriptor v,DfsGraph& g)
{
  std::cout << "discover " << get(vertex_name,g,v) << std::endl;
//  std::cout << "out_degree(v,g)= " << out_degree(v,g) << std::endl;
}
  
template <class DfsGraph>
void tree_edge(typename boost::graph_traits<DfsGraph>::edge_descriptor e,DfsGraph& g)
{
//  cerr << "Adding: " << get(edge_name,g,e) << endl;
  hot_path_.push_back(e);
  
  cerr << "hot_path_: ";
  for(typename std::vector<typename boost::graph_traits<DfsGraph>::edge_descriptor>::const_iterator i=hot_path_.begin(); i!=hot_path_.end(); ++i)
    cout << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
  cout << endl;
  
  get_samples_from_hot_path<DfsGraph>(g);
}

template <class DfsGraph>
void 
finish_vertex(typename boost::graph_traits<DfsGraph>::vertex_descriptor v,DfsGraph& g)
{
  std::cerr << "finish " << get(vertex_name,g,v) << std::endl;  
  hot_path_.erase( hot_path_.end()-1 );

  // This is to make the vertex named TidyHome to be discovered again if there is another solution path
  std::string name;
  name = get(vertex_name,g,v);

  if( !strcmp(name.c_str(),"TidyHome") )
    put(vertex_color,g,v,color_traits<boost::default_color_type>::white());
}

bool
get_fval(const std::vector<typename boost::graph_traits<Graph>::edge_descriptor>& path,const Graph& g,const string& metadata_path,Input* in)
{
  labels_ = get_labels(metadata_path);
  
  return get_fval<Graph>(path,g,in);
}

private:
//! Extract a sample from a path
/*!
  More ...
*/
template<typename LocalGraph>
bool
get_sample(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path,const LocalGraph& g)
{
  Input in;
  if( !get_fval<LocalGraph>(path,g,&in) )
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

  data_->insert( std::make_pair(in,out) );
  
  return true;
}

template<typename LocalGraph>
void
get_samples_from_hot_path(const LocalGraph& g)
{
  cerr << "hot_path_.size()= " << hot_path_.size() << endl;
  
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::iterator i=hot_path_.begin(); i!=hot_path_.end(); ++i)
  {
    std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor> path(i,hot_path_.end());
    
    if( !get_sample<LocalGraph>(path,g) )
      continue;
  }
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
  
  // Scaling
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
get_fval(const std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>& path, const LocalGraph& g,Input* in)
{
//  cerr << "path: ";
//  for(std::vector<typename boost::graph_traits<Graph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
//    cerr << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
//  cerr << endl;
  
  RawInput r_in;
  
  // Geo. features extracted from the head vertex of this path: object pose+manipulability+jstates in the source vertex
  std::string srcstate;
  srcstate = get( edge_srcstate,g,path.at(0) );
  
  if( !get_geo_fval(srcstate,&r_in) )
    return false;
      
  // Symbolic features: Whether more TRANSFER or TRANSIT from actions in this path
  // Symbolic features: Whether more LARM or RARM from actions in this path
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

  r_in.insert( std::make_pair("TRANSIT-centric",(n_transit > n_transfer)) );
  r_in.insert( std::make_pair("TRANSFER-centric",(n_transfer > n_transit)) );
  
  r_in.insert( std::make_pair("RARM-centric",(n_rarm > n_larm)) );
  r_in.insert( std::make_pair("LARM-centric",(n_larm > n_rarm)) );
  
  // Symbolic features: Position of certain action label in a path
  for(typename std::vector<typename boost::graph_traits<LocalGraph>::edge_descriptor>::const_iterator i=path.begin(); i!=path.end(); ++i)
  {
    std::string name;
    name = get(edge_name,g,*i);
    
    size_t idx;
    idx = i - path.begin() + 1;// Plus one because idx=0 is reserved for if Act_i is not in this path
    
    r_in.insert( std::make_pair(name,(double)idx) );
  }
  
  cerr << "r_in.size()= " << r_in.size() << endl;
//    for(RawInput::iterator z=r_in.begin(); z!=r_in.end(); ++z)
//      cerr << z->first << "= " << z->second << endl;
//    cerr << "y= " << out << endl;
  
  // Convert then return
  return convert( r_in,in,labels_ );
}

bool 
get_geo_fval(const std::string& srcstate,RawInput* r_in,const std::string& suffix="")
{
  //(1) object pose in the source vertex
  std::vector<std::string> srcstate_parts;
  boost::split( srcstate_parts, srcstate, boost::is_any_of(";") );
  
  if( srcstate_parts.at(0).size() == 0 )
  {
    cerr << "srcstate_parts.at(0).size() == 0" << endl;
    return false;
  }

  std::map<std::string,double> jname_jpos_map;// for obtaining jstate later on
  
  for(std::vector<std::string>::const_iterator i=srcstate_parts.begin(); i!=srcstate_parts.end(); ++i )
  {
    std::vector<std::string> comps;
    boost::split( comps, *i, boost::is_any_of(",") );
    
    if(comps.size()==8)// : id, x, y, z, qx, qy, qz, qw
    {
      std::string obj_id = comps.at(0);
      comps.erase(comps.begin());// to make comps and names (below) exactly have 7 elements
      
      std::vector<std::string> names;
      names.push_back(obj_id+".x"+suffix);
      names.push_back(obj_id+".y"+suffix);
      names.push_back(obj_id+".z"+suffix);
      names.push_back(obj_id+".qx"+suffix);
      names.push_back(obj_id+".qy"+suffix);
      names.push_back(obj_id+".qz"+suffix);
      names.push_back(obj_id+".qw"+suffix);
      
      for(std::vector<std::string>::const_iterator j=names.begin(); j!=names.end(); ++j)
      {
        std::string name = *j;
        double val = boost::lexical_cast<double>( comps.at(j-names.begin()) );
        
        r_in->insert( std::make_pair(name,val) );
      }
    }
    else if(comps.size()==2)// joint-name, joint-state
    {
      // Assume that no duplicated joint data 
      jname_jpos_map[comps.at(0)] = boost::lexical_cast<double>(comps.at(1));
    }
    else
    {
      ROS_ERROR_STREAM("srcstate is corrupt; comps.size()= " << comps.size() );
      return false;
    }
  }

  //(2) joint-state on the source vertex
  for(std::map<std::string,double>::const_iterator i=jname_jpos_map.begin(); i!=jname_jpos_map.end(); ++i)
  {
    r_in->insert(  std::make_pair( i->first,i->second )  );
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

Data* data_;
Input* in_;

std::vector<typename boost::graph_traits<Graph>::edge_descriptor> hot_path_;
std::vector<std::string> labels_;
};

#endif // #ifndef DATA_COLLECTOR_HPP_INCLUDED
