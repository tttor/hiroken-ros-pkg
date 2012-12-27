#ifndef DATA_COLLECTOR_HPP_INCLUDED
#define DATA_COLLECTOR_HPP_INCLUDED

#include "data_collector.hpp"
#include "tmm_utils.hpp"
#include "data.hpp"

#include <boost/graph/depth_first_search.hpp>

struct DataCollectorFoundGoalSignal {}; // exception for termination

typedef enum 
{
  IO_OFFLINE=1, 
  IO_ONLINE, 
  IN_ONLINE
} Mode;

class DataCollector: public boost::dfs_visitor<>
{
public:
DataCollector(Data* data,std::vector<std::string> feature_names)
: data_(data)
, feature_names_(feature_names)
{ 
  mode_ = IO_OFFLINE;
}

DataCollector(Data* data,std::string metadata_path,TMMVertex goal)
: data_(data)
, metadata_path_(metadata_path)
, goal_(goal)
{ 
  mode_ = IO_ONLINE;
  feature_names_ = get_feature_names();
  
  in_ = 0;// null pointer
}

DataCollector(Input* in,std::string metadata_path,TMMVertex goal)
: in_(in)
, metadata_path_(metadata_path)
, goal_(goal)
{
  mode_ = IN_ONLINE;
  feature_names_ = get_feature_names();
  
  data_ = 0;// null pointer
}

template <class Graph>
void 
discover_vertex(typename boost::graph_traits<Graph>::vertex_descriptor v,Graph& g)
{
  std::cout << "discover " << get(vertex_name,g,v) << std::endl;
  std::cout << "out_degree(v,g)= " << out_degree(v,g) << std::endl;
  
  if(v==goal_)
  {
    switch(mode_)
    {
      case IO_ONLINE:
      {
        get_samples<Graph>(g);
        break;
      }
      case IO_OFFLINE:
      {
        break;
      }
      case IN_ONLINE:
      {
        *in_ = get_in(hot_path_,g);
        break;
      }
    }
    
    cerr << "throwing goal sgn" << endl;
    throw DataCollectorFoundGoalSignal(); 
  }
}
  
template <class Graph>
void tree_edge(typename boost::graph_traits<Graph>::edge_descriptor e,Graph& g)
{
//  cerr << "Adding: " << get(edge_name,g,e) << endl;
  hot_path_.push_back(e);
  
  cerr << "hot_path_: ";
  for(std::vector<TMMEdge>::const_iterator i=hot_path_.begin(); i!=hot_path_.end(); ++i)
    cout << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
  cout << endl;
  
  if(mode_==IO_OFFLINE)
    get_samples<Graph>(g);
}

template <class Graph>
void 
finish_vertex(typename boost::graph_traits<Graph>::vertex_descriptor v,Graph& g)
{
  std::cout << "finish " << get(vertex_name,g,v) << std::endl;  
  hot_path_.erase( hot_path_.end()-1 );
}
  
private:
Input
convert(RawInput r_in)
{
  Input in;
  
  for(std::vector<std::string>::const_iterator i=feature_names_.begin(); i!=feature_names_.end(); ++i)
  {
    RawInput::iterator j;
    j = r_in.find(*i);
    
    if( j!=r_in.end() )
      in.push_back( j->second );
    else
      in.push_back(0.);// Make it a point and at the Origin for object's pose; Set not-exist value for symbolic features
  }
  
  return in;
}
//! This gets the upper bound of input-feature size
/*!
  More ...
*/
std::vector<std::string>
get_feature_names()
{
  std::vector<std::string> feature_names;
  
  // Read from a csv file containing metadata
  std::ifstream metadata_in(metadata_path_.c_str());
  
  if ( metadata_in.is_open() )
  {
    std::string metadata;
    
    getline(metadata_in, metadata);// Read only the first line; the only line here
    metadata_in.close();
    
    // Parse the metadata, put into a vector
    boost::split( feature_names, metadata, boost::is_any_of(",") );

    // Note that eventhough there is no "," or the metadata is empty, the resulted vector still has 1 element which is an empty string.
    if( !strcmp(feature_names.at(0).c_str(), std::string("").c_str())  )
    {
        std::cerr << "metadata file is corrupt." << endl;
        feature_names.erase(feature_names.begin());
    }
    else
    {
      // Remove the OUT label
      std::vector<std::string>::iterator OUT_it;
      OUT_it = std::find(feature_names.begin(), feature_names.end(), "OUT");
      if(OUT_it!=feature_names.end())
        feature_names.erase(OUT_it);
    }
  }
  else 
  {
    std::cerr << "Unable to open metadata file" << endl;
  }
  
  return feature_names;
}
//! Extract samples from hot_path_
/*!
  More ...
*/
template<typename Graph>
void
get_samples(const Graph& g)
{
  cerr << "hot_path_.size()= " << hot_path_.size() << endl;
  
  for(std::vector<TMMEdge>::iterator i=hot_path_.begin(); i!=hot_path_.end(); ++i)
  {
    std::vector<TMMEdge> dummy_heu_path(i,hot_path_.end());
    cerr << "dummy_heu_path.size()= " << dummy_heu_path.size() << endl;

    Input in;
    in = get_in(dummy_heu_path,g);
    
    Output out;
    out = get_out(dummy_heu_path,g);
    
    data_->insert( std::make_pair(in,out) );
  }// end of: for each edge in heu_path, which is a subset of hot_path_
  
}

template<typename Graph>
Output
get_out(const std::vector<TMMEdge>& heu_path, const Graph& g)
{
  Output out = 0.;
  
  for(std::vector<TMMEdge>::const_iterator j=heu_path.begin(); j!=heu_path.end(); ++j)
  {
    out += get(edge_weight, g, *j);
  }
  
  return out;
}
//! Extract input feature vector from a heuristics path
/*!
  (1)feature-input: 
  (1.1) symbolic features: position of an action Acti in the heu_path
  (1.2) geometric features: state descriptions, namely poses of movable objects, jstates
  
  Depends on the mode_
  If IO_OFFLINE, read_graphviz can not recover vertex properties, therefore geo. feature is obtained from edge_property: srcstate
  
  TODO make the types consistent
*/
template<typename Graph>
Input
get_in(const std::vector<TMMEdge>& heu_path, const Graph& g)
{
  cerr << "heu_path_: ";
  for(std::vector<TMMEdge>::const_iterator i=heu_path.begin(); i!=heu_path.end(); ++i)
    cerr << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
  cerr << endl;
  
  Input in;
  RawInput r_in;
  
  // Symbolic features
  for(std::vector<TMMEdge>::const_iterator j=heu_path.begin(); j!=heu_path.end(); ++j)
  {
    std::string name;
    name = get(edge_name, g, *j);
    
    size_t idx;
    idx = j - heu_path.begin() + 1;// Plus one because idx=0 is reserved for if Act_i is not in this heu_path
    
    r_in.insert( std::make_pair(name,(double)idx) );
  }
  
  // Geometric features
  if(mode_==IO_OFFLINE)// then, poses of movable objects in source vertex of the first edge in heu_path
  {
    std::string state_str;
    state_str = get(edge_srcstate,g,heu_path.front());
    
    std::vector<std::string> state_str_parts;
    boost::split( state_str_parts, state_str, boost::is_any_of(";") );
    
    for(std::vector<std::string>::const_iterator i=state_str_parts.begin(); i!=state_str_parts.end(); ++i )
    {
      std::vector<std::string> comps;
      boost::split( comps, *i, boost::is_any_of(",") );
      
      if(comps.size()==8)// : id, x, y, z, qx, qy, qz, qw
      {
        std::string obj_id = comps.at(0);
        comps.erase(comps.begin());// to make comps and names exactly matched
        
        std::vector<std::string> names;
        names.push_back(obj_id+".x");
        names.push_back(obj_id+".y");
        names.push_back(obj_id+".z");
        names.push_back(obj_id+".qx");
        names.push_back(obj_id+".qy");
        names.push_back(obj_id+".qz");
        names.push_back(obj_id+".qw");
        
        for(std::vector<std::string>::const_iterator j=names.begin(); j!=names.end(); ++j)
        {
          std::string name = *j;
          double val = boost::lexical_cast<double>( comps.at(j-names.begin()) );
          
          r_in.insert( std::make_pair(name,val) );
        }
      }
      else if(comps.size()==2)// joint-name, joint-state
      {
        r_in.insert(  std::make_pair( comps.at(0),boost::lexical_cast<double>(comps.at(1)) )  );
      }
      else
      {
        std::cerr << "state_str is corrupt; comps.size()= " << comps.size() << std::endl;
        return in;// empty!
      }
    }
  }
  else if( (mode_==IN_ONLINE)or(mode_==IO_ONLINE) )// then, get geo. feature from head vertex prop.
  {
    std::vector<arm_navigation_msgs::CollisionObject> wstate;
    wstate = get( vertex_wstate,g,source(heu_path.front(),g) );
    
    for(std::vector<arm_navigation_msgs::CollisionObject>::const_iterator i=wstate.begin(); i!=wstate.end(); ++i)
    {
      r_in.insert(  std::make_pair( std::string(i->id+".x"),i->poses.at(0).position.x )  );
      r_in.insert(  std::make_pair( std::string(i->id+".y"),i->poses.at(0).position.y )  );
      r_in.insert(  std::make_pair( std::string(i->id+".z"),i->poses.at(0).position.z )  );
      r_in.insert(  std::make_pair( std::string(i->id+".qx"),i->poses.at(0).orientation.x )  );
      r_in.insert(  std::make_pair( std::string(i->id+".qy"),i->poses.at(0).orientation.y )  );
      r_in.insert(  std::make_pair( std::string(i->id+".qz"),i->poses.at(0).orientation.z )  );
      r_in.insert(  std::make_pair( std::string(i->id+".qw"),i->poses.at(0).orientation.w )  );
    }
    
    sensor_msgs::JointState jstate;
    jstate = get( vertex_jstates,g,source(heu_path.front(),g) );
    
    for(std::vector<std::string>::const_iterator i=jstate.name.begin(); i!=jstate.name.end(); ++i)
    {
      r_in.insert(  std::make_pair( *i,jstate.position.at(i-jstate.name.begin()) )  );
    }
  }
  
  cerr << "r_in.size()= " << r_in.size() << endl;
//    for(RawInput::iterator z=r_in.begin(); z!=r_in.end(); ++z)
//      cerr << z->first << "= " << z->second << endl;
//    cerr << "y= " << out << endl;
  
  // Convert r_in to in
  in = convert(r_in);
  
  return in;
}

Data* data_;
Input* in_;
std::vector<TMMEdge> hot_path_;

std::vector<std::string> feature_names_;
std::string metadata_path_;
std::string data_path_;

TMMVertex goal_;
bool with_goal_;// because we are not sure what default val for TMMVertex is

Mode mode_;
};

#endif // #ifndef DATA_COLLECTOR_HPP_INCLUDED
