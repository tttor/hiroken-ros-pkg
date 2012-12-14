#ifndef DATA_COLLECTOR_HPP_INCLUDED
#define DATA_COLLECTOR_HPP_INCLUDED

#include <boost/graph/depth_first_search.hpp>

#include "data_collector.hpp"
#include "tmm_utils.hpp"

typedef double Output;// which is the true geometric cost
typedef std::vector<double> Input;// which consists of feature-value, whose name is specified in the metadata
typedef std::map<std::string, double> RawInput;// which consists of features-name--feature-value pairs
typedef std::map<Input, Output> Data;

class DataCollector: public boost::dfs_visitor<>
{
public:
DataCollector(Data* data,std::vector<std::string> feature_names)
: data_(data)
, feature_names_(feature_names)
{ }

//template <class Graph>
//void 
//discover_vertex(typename boost::graph_traits<Graph>::vertex_descriptor v,Graph& g)
//{
//  std::cout << "discover " << get(vertex_name,g,v) << std::endl;
//}
  
template <class Graph>
void tree_edge(typename boost::graph_traits<Graph>::edge_descriptor e,Graph& g)
{
//  cerr << "Adding: " << get(edge_name,g,e) << endl;
  hot_edges_.push_back(e);
  
  cerr << "hot_edges_: ";
  for(std::vector<TMMEdge>::const_iterator i=hot_edges_.begin(); i!=hot_edges_.end(); ++i)
    cout << "e(" << get(vertex_name,g,source(*i,g)) << "," << get(vertex_name,g,target(*i,g)) << "), ";
  cout << endl;
    
  // Obtain below values from a heuristics path
  // (1)feature-input: 
  // (1.1) symbolic features: position of an action Acti in the heu_path
  // (1.2) geometric features: state descriptions, namely poses of movable objects, jstates
  // (2)output
  for(std::vector<TMMEdge>::iterator i=hot_edges_.begin(); i!=hot_edges_.end(); ++i)
  {
    std::vector<TMMEdge> heu_path(i,hot_edges_.end());
    cerr << "heu_path.size()= " << heu_path.size() << endl;

    RawInput r_in;
    Output out = 0.;
    
    // Symbolic features + obtain the true output value
    for(std::vector<TMMEdge>::const_iterator j=heu_path.begin(); j!=heu_path.end(); ++j)
    {
      std::string name;
      name = get(edge_name, g, *j);
      
      size_t idx;
      idx = j - heu_path.begin() + 1;// Plus one because idx=0 is reserved for if Act_i is not in this heu_path
      
      r_in.insert( std::make_pair(name,(double)idx) );
      
      out += get(edge_weight, g, *j);
    }
    
    // Geometric features
    // ::Poses of movable objects in source vertex of the first edge in heu_path
    std::string state_str;
    state_str = get(edge_srcstate,g,heu_path.front());
    
    std::vector<std::string> state_str_parts;
    boost::split( state_str_parts, state_str, boost::is_any_of(";") );
    
    for(std::vector<std::string>::const_iterator i=state_str_parts.begin(); i!=state_str_parts.end(); ++i )
    {
      std::vector<std::string> comps;
      boost::split( comps, *i, boost::is_any_of(",") );
      
      if(comps.size()!=8)// : id, x, y, z, qx, qy, qz, qw
      {
        std::cerr << "comps.size()= " << comps.size() << std::endl;
        std::cerr << "state_str is corrupt!" << std::endl;
        return;
      }
      
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
    
//    for(RawInput::iterator z=r_in.begin(); z!=r_in.end(); ++z)
//    {
//      cerr << z->first << "= " << z->second << endl;
//    }
//    cerr << "y= " << out << endl;
    
    // Convert r_in to in
    Input in;
    convert(r_in,&in);
    
    data_->insert( std::make_pair(in,out) );
  }// end of: for each edge in heu_path, which is a subset of hot_edges_
}

template <class Graph>
void 
finish_vertex(typename boost::graph_traits<Graph>::vertex_descriptor v,Graph&)
{
  hot_edges_.erase( hot_edges_.end()-1 );
}
  
private:
void
convert(RawInput r_in, Input* in)
{
  for(std::vector<std::string>::const_iterator i=feature_names_.begin(); i!=feature_names_.end(); ++i)
  {
    RawInput::iterator j;
    j = r_in.find(*i);
    
    if( j!=r_in.end() )
      in->push_back( j->second );
    else
      in->push_back(0.);// Make it a point and at the Origin
  }
}

Data* data_;
std::vector<TMMEdge> hot_edges_;
std::vector<std::string> feature_names_;
};

#endif // #ifndef DATA_COLLECTOR_HPP_INCLUDED
