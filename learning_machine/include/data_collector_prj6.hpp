#ifndef DATA_COLLECTOR_PRJ6_HPP_INCLUDED
#define DATA_COLLECTOR_PRJ6_HPP_INCLUDED

#include "data_collector_prj6.hpp"
#include "data.hpp"
#include "tmm_utils.hpp"
#include "data_collector_common.hpp"

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

namespace dc_prj6
{

static const size_t g_np = 100;

//! This is hardcoded a lot!
void
create_metadata(std::string metadata_path)
{
  std::ofstream metadata;
  metadata.open(metadata_path.c_str());
  
  size_t n_feature = 0;

  // objects' pose
  std::vector<std::string> obj_ids;
  obj_ids.push_back("unmovable.table");
  obj_ids.push_back("unmovable.VASE1");
  obj_ids.push_back("unmovable.VASE2");
  obj_ids.push_back("CAN1");
  
  for(std::vector<std::string>::const_iterator i=obj_ids.begin(); i!=obj_ids.end(); ++i)
  {
    metadata << std::string(*i+".x") << "," 
             << std::string(*i+".y") << "," 
             << std::string(*i+".z") << "," 
             << std::string(*i+".qx") << "," 
             << std::string(*i+".qy") << "," 
             << std::string(*i+".qz") << "," 
             << std::string(*i+".qw") << ",";
  }
  
  n_feature += 7;
  
  // Qs and Qg and Q-waypoint data 
  std::vector<std::string> q_ids;
  q_ids.push_back("qs");
  q_ids.push_back("qg");
  
  std::string qw_base_id = "qw";
  for(size_t i=0; i<g_np; ++i)
  {
    q_ids.push_back( std::string(qw_base_id+boost::lexical_cast<std::string>(i+1)) );
  }

  for(std::vector<std::string>::const_iterator i=q_ids.begin(); i!=q_ids.end(); ++i)
  {
    metadata << std::string(*i+".joint_chest_yaw") << ","
             << std::string(*i+".joint_relbow_pitch") << ","
             << std::string(*i+".joint_rshoulder_pitch") << ","
             << std::string(*i+".joint_rshoulder_yaw") << ","
             << std::string(*i+".joint_rwrist_pitch") << ","
             << std::string(*i+".joint_rwrist_roll") << ",";
    
    if(i != q_ids.end()-1) 
      metadata << std::string(*i+".joint_rwrist_yaw") << ",";
    else
      metadata << std::string(*i+".joint_rwrist_yaw");
  }
  
  n_feature += (2+g_np)*7;
  
  // Closure
  std::cerr << "n_feature= " << n_feature << endl;
  metadata.close();
}

class DataCollector
{
public:
DataCollector(std::string metadata_path)
: metadata_path_(metadata_path)
{ 
  boost::mt19937 rng( static_cast<unsigned int>(std::time(0)) );
  rng_ = rng;
}

~DataCollector()
{ }

bool
get_fval(const TaskMotionMultigraph& tmm)
{
  std::cerr << "get_fval()..." << std::endl;
  
  // For now, only "tranfer" edge is eligible
  std::list<TMMEdge> e_list;

  boost::graph_traits<TaskMotionMultigraph>::edge_iterator ei, ei_end;
  for(boost::tie(ei,ei_end) = edges(tmm); ei!=ei_end; ++ei)
  {
    std::string label;
    label = get(edge_name,tmm,*ei);
    
    std::vector<std::string> label_parts;
    boost::split( label_parts,label,boost::is_any_of("_") );
    
    std::string motion_type;
    motion_type = label_parts.at(0);
    
    if( !strcmp(motion_type.c_str(),"TRANSFER") )
    {
      e_list.push_back(*ei);
      break;
    }
  }
  
  for(std::list<TMMEdge>::const_iterator i=e_list.begin(); i!=e_list.end(); ++i)
  {
    RawInput raw_in;
    
    // Extract workspace representation: object pose + qs, which is in the source vertex (srcstate) of the edge e
    if( !dc_common::get_geo_fval( get(edge_srcstate,tmm,*i),&raw_in ) )
      continue;
    
    // Extract Q_g, trajectory representation
    if( !get_traj_fval( get(edge_planstr,tmm,*i),&raw_in ) )
      continue;

//  for(RawInput::const_iterator i=raw_in_.begin(); i!=raw_in_.end(); ++i)
//  {
//    std::cerr << i->first << " -> " <<  i->second << std::endl;
//  }
    
    // Convert and put to the data
    Input in;
    convert( raw_in,&in,get_labels(metadata_path_) );  
  
    data_.push_back(in);
  }
  
  return true;
}

bool
write(const std::string& data_path)
{
  data_utils::write_csv(data_,data_path);
  
  return true;
}

private:
bool
get_traj_fval(const std::string& planstr,RawInput* r_in)
{
  std::cerr << "get_traj_fval()..." << std::endl;

  trajectory_msgs::JointTrajectory traj;
  traj = utils::get_plan(planstr);

  if(traj.points.empty())
  {
    ROS_WARN("traj.points.empty()");
    return false;
  }
  
  // For the final robot configuration, note the the start configuration is obtained in get_geo_fval() using srcstate
  for(size_t i=0; i<traj.joint_names.size(); ++i)
  {
    std::string label = std::string("qg."+traj.joint_names.at(i));
    double value = traj.points.back().positions.at(i);
    
    r_in->insert( std::make_pair(label,value) );
  }
  
  // Extract trajectory representation
  std::vector<size_t> rep_idxes;
  if( !get_traj_rep(traj,&rep_idxes) )
    return false;
  
  for(std::vector<size_t>::const_iterator i=rep_idxes.begin(); i!=rep_idxes.end(); ++i)
  {
    size_t ith = i-rep_idxes.begin();
    
    for(size_t j=0; j<traj.joint_names.size(); ++j)
    {
      std::string label = std::string("qw"+boost::lexical_cast<std::string>(ith+1)+"."+traj.joint_names.at(j));
      double value = traj.points.at(*i).positions.at(j);
      
      r_in->insert( std::make_pair(label,value) );
    }
  }
  
  return true;
}

//! Obtain a representation of a trajectory
/*!
Method: 
Given a number of desired waypoint, g_np, to represent a trajectory,
this return indexes of g_np waypoint, excluding the start and the final robot configurations.
*/
bool
get_traj_rep(const trajectory_msgs::JointTrajectory& traj,std::vector<size_t>* chosen_idxes)
{
  std::cerr << "get_traj_rep()..." << std::endl;
  std::cerr << "traj.points.size()= " << traj.points.size() << std::endl;
  
  size_t np = g_np;  
//  if(np > (traj.points.size()-2))// minus qs and qg points
//    np = (traj.points.size()-2);
  if(np > (traj.points.size()-2))// minus qs and qg points
  {
    std::cerr << "lack of waypoints" << std::endl;
    return false;
  }
  
  std::set<size_t> chosen_idxes_set;
  for(size_t i=0; i<g_np; ++i)
  {
    size_t idx;
    bool passed = false;
    do
    {
      boost::uniform_int<> uni_int_dist( 1,traj.points.size()-1-1 );// range = [a,b]
      idx = uni_int_dist(rng_);
      
      std::pair<std::set<size_t>::iterator,bool> ret;
      ret = chosen_idxes_set.insert(idx);
      
      passed = ret.second;
    }
    while(!passed);
    
    chosen_idxes->push_back(idx);
  }
  std::sort(chosen_idxes->begin(),chosen_idxes->end());// the order is preserved: ascending
  
//  std::cerr << "chosen_idxes= ";
//  for(size_t i=0; i<chosen_idxes.size(); ++i)
//    std::cerr << chosen_idxes.at(i) << " ";
//  std::cerr << std::endl;
  
  return true;
}

//! Keeps input-only data 
InputOnlyData data_;
//! For random number generator
boost::mt19937 rng_;
//! ...
std::string metadata_path_;
};

}// namespace dc_prj6

#endif // #ifndef DATA_COLLECTOR_PRJ6_HPP_INCLUDED
