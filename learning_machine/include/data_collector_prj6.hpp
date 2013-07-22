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
  std::vector<std::string> labels;
  
  // 1. Obstacle position
  labels.push_back("unmovable.VASE1.i.x");
  labels.push_back("unmovable.VASE1.i.y");
  
  // 2. Movable object initial pose 
  labels.push_back("CAN1.i.x");
  labels.push_back("CAN1.i.y");
  labels.push_back("CAN1.i.z");
  labels.push_back("CAN1.i.qx");
  labels.push_back("CAN1.i.qy");
  labels.push_back("CAN1.i.qz");
  labels.push_back("CAN1.i.qw");
  
  // 3. Movable object final position
  labels.push_back("CAN1.f.x");
  labels.push_back("CAN1.f.y");
  
  // 4. Robot start pose, at GRASPED_CAN1
  labels.push_back("joint_chest_yaw.i");
  labels.push_back("joint_rshoulder_yaw.i");
  labels.push_back("joint_rshoulder_pitch.i");
  labels.push_back("joint_relbow_pitch.i");
  labels.push_back("joint_rwrist_yaw.i");
  labels.push_back("joint_rwrist_pitch.i");
  labels.push_back("joint_rwrist_roll.i");
  labels.push_back("RARM.m.i");

  // 5. Robot final pose, at RELEASED_CAN1
  labels.push_back("joint_chest_yaw.f");
  labels.push_back("joint_rshoulder_yaw.f");
  labels.push_back("joint_rshoulder_pitch.f");
  labels.push_back("joint_relbow_pitch.f");
  labels.push_back("joint_rwrist_yaw.f");
  labels.push_back("joint_rwrist_pitch.f");
  labels.push_back("joint_rwrist_roll.f");
  labels.push_back("RARM.m.f");
  
  // 6. For Ground truth, expected cluster
  labels.push_back("OUT");
  
  // Write
  std::ofstream metadata;
  metadata.open(metadata_path.c_str());
  
  for(size_t i=0; i<labels.size(); ++i)
  {
    metadata << labels.at(i);
    
    if(i < labels.size()-1 )
      metadata << ",";
  }
  metadata.close();
  
  cerr << "metadata creates with n_feature= " << labels.size() << endl;
}


class DataCollector
{
public:
DataCollector(std::string metadata_path)
: metadata_path_(metadata_path)
{  }

~DataCollector()
{ }

bool
get_out(const std::string& sol_tmm_path,Output* out)
{
  // Parsing the path to get the expected cluster, e.g. from /home/vektor/prj6/data/baseline/c1.20130721.a.2/sol_tmm.dot
  std::vector<std::string> path_parts;
  boost::split( path_parts,sol_tmm_path,boost::is_any_of("/") );
  
  std::string hot_part;
  hot_part = path_parts.at( path_parts.size()-2 );
  
  std::string cluster_str;
  cluster_str = hot_part.substr(1,1);
  
  *out = boost::lexical_cast<double>(cluster_str);
  
  return true;
}

bool
get_fval(const TaskMotionMultigraph& sol_tmm,Input* in)
{
  // For now, only "tranfer" edge is eligible
  TMMEdge hot_edge;// must be only one hot edge from a sol path tmm
  TMMEdge hot_edge_2; // the out edge of hot_edge's target vertex

  boost::graph_traits<TaskMotionMultigraph>::edge_iterator ei, ei_end;
  for(boost::tie(ei,ei_end) = edges(sol_tmm); ei!=ei_end; ++ei)
  {
    std::string label;
    label = get(edge_name,sol_tmm,*ei);
    
    std::vector<std::string> label_parts;
    boost::split( label_parts,label,boost::is_any_of("_") );
    
    std::string motion_type;
    motion_type = label_parts.at(0);
    
    if( !strcmp(motion_type.c_str(),"TRANSFER") )
    {
      hot_edge = *ei;    
      
      TMMVertex hot_vertex;
      hot_vertex = target(hot_edge,sol_tmm);
      
      hot_edge_2 = *( out_edges(hot_vertex,sol_tmm).first );
      
      break;
    }
  }  

  // Init: parsing string of edge_srcstate 
  std::string srcstate;
  std::vector<std::string> srcstate_parts;
  
  std::map<std::string,double> jname_jpos_map;
  std::map<std::string,double> effector_manipulability_map;
  std::map< std::string,std::vector<double> > obj_pose_map;
  
  // (1) ... of hot_edge
  srcstate = get( edge_srcstate,sol_tmm,hot_edge );
  
  boost::split( srcstate_parts, srcstate, boost::is_any_of(";") );
  if( srcstate_parts.at(0).size() == 0 )
  {
    cerr << "srcstate_parts.at(0).size() == 0 -> get_geo_fval() returns false" << endl;
    return false;
  }

  for(std::vector<std::string>::const_iterator i=srcstate_parts.begin(); i!=srcstate_parts.end(); ++i)
  {
    std::vector<std::string> comps;
    boost::split( comps, *i, boost::is_any_of(",") );
    
    std::string header;
    header = comps.at(0);
    comps.erase( comps.begin() );
    
    if( !strcmp(header.c_str(),std::string("obj_pose").c_str()) )
    {
      std::string id = std::string( comps.at(0)+".i" );// initial pose
      comps.erase( comps.begin() );
      
      std::vector<double> pose(7);// object's pose data: x, y, z, qx, qy, qz, qw
      for(size_t j=0; j<7; ++j)
      {
        pose.at(j) = boost::lexical_cast<double>( comps.at(j) );
      }
      
      obj_pose_map[id] = pose;
    }
    else if( !strcmp(header.c_str(),std::string("jstate").c_str()) )
    {
      std::string jname;
      jname = std::string( comps.at(0)+".i" );
      
      double jstate;
      jstate = boost::lexical_cast<double>( comps.at(1) );
      
      jname_jpos_map[jname] = jstate;
    }
    else if( !strcmp(header.c_str(),std::string("manipulability").c_str()) )
    {
      std::string effector;
      effector = std::string( comps.at(0)+".m.i" );
      
      double m;
      m = boost::lexical_cast<double>( comps.at(1) );
      
      effector_manipulability_map[effector] = m;
    }
    else
    {
      cerr << "[ERROR] srcstate is corrupt; srcstate: " << srcstate << endl;
      return false;
    }
  }
  // (2) ... of hot_edge _2
  srcstate = get( edge_srcstate,sol_tmm,hot_edge_2 );
  
  boost::split( srcstate_parts, srcstate, boost::is_any_of(";") );
  if( srcstate_parts.at(0).size() == 0 )
  {
    cerr << "srcstate_parts.at(0).size() == 0 -> get_geo_fval() returns false" << endl;
    return false;
  }

  for(std::vector<std::string>::const_iterator i=srcstate_parts.begin(); i!=srcstate_parts.end(); ++i)
  {
    std::vector<std::string> comps;
    boost::split( comps, *i, boost::is_any_of(",") );
    
    std::string header;
    header = comps.at(0);
    comps.erase( comps.begin() );
    
    if( !strcmp(header.c_str(),std::string("obj_pose").c_str()) )
    {
      std::string id = std::string( comps.at(0)+".f" );// initial pose
      comps.erase( comps.begin() );
      
      std::vector<double> pose(7);// object's pose data: x, y, z, qx, qy, qz, qw
      for(size_t j=0; j<7; ++j)
      {
        pose.at(j) = boost::lexical_cast<double>( comps.at(j) );
      }
      
      obj_pose_map[id] = pose;
    }
    else if( !strcmp(header.c_str(),std::string("jstate").c_str()) )
    {
      std::string jname;
      jname = std::string( comps.at(0)+".f" );
      
      double jstate;
      jstate = boost::lexical_cast<double>( comps.at(1) );
      
      jname_jpos_map[jname] = jstate;
    }
    else if( !strcmp(header.c_str(),std::string("manipulability").c_str()) )
    {
      std::string effector;
      effector = std::string( comps.at(0)+".m.f" );
      
      double m;
      m = boost::lexical_cast<double>( comps.at(1) );
      
      effector_manipulability_map[effector] = m;
    }
    else
    {
      cerr << "[ERROR] srcstate is corrupt; srcstate: " << srcstate << endl;
      return false;
    }
  }

  // DO Extract
  RawInput raw_in;
  
  // 1. Obstacle position from the srcstate property of the hot edge
  // 2. Movable object initial pose from the srcstate property of the hot edge
  // 3. Movable object final position, from the out edge's srcstate of hot_edge's target vertex i.e. hot_edge_2, here, out edge of vertex "RELEASED_CAN1"
  for(std::map< std::string,std::vector<double> >::const_iterator i=obj_pose_map.begin(); i!=obj_pose_map.end(); ++i)
  {
    std::string obj_id;
    obj_id = i->first;
      
    std::vector<std::string> labels(7);
    labels.at(0) = obj_id+".x";
    labels.at(1) = obj_id+".y";
    labels.at(2) = obj_id+".z";
    labels.at(3) = obj_id+".qx";
    labels.at(4) = obj_id+".qy";
    labels.at(5) = obj_id+".qz";
    labels.at(6) = obj_id+".qw";
      
    for(size_t j=0; j<labels.size(); ++j)
    {
      std::string label;
      label = labels.at(j);
      
      double val;
      val = i->second.at(j);
      
      raw_in.insert( std::make_pair(label,val) );
    }
  }
  
  // 4. Robot start pose and its manipulability, at GRASPED_CAN1 from the srcstate property of the hot edge
  // 5. Robot final pose and its manipulability, at RELEASED_CAN1
  for(std::map<std::string,double>::const_iterator i=jname_jpos_map.begin(); i!=jname_jpos_map.end(); ++i)
  {
    raw_in.insert(  std::make_pair( i->first,i->second )  );
  }
  for(std::map<std::string,double>::const_iterator i=effector_manipulability_map.begin(); i!=effector_manipulability_map.end(); ++i)
  {
    raw_in.insert(  std::make_pair( i->first,i->second )  );
  }
  

  // Convert
//  cerr << "raw_in= " << endl;
//  for(RawInput::const_iterator i=raw_in.begin(); i!=raw_in.end(); ++i)
//    cerr << i->first << " --> " << i->second << endl;
  
  data_util::convert( raw_in,in,data_util::get_labels(metadata_path_) );  

//  cerr << "in.size()= " << in->size() <<  endl;
//  for(Input::const_iterator i=in->begin(); i!=in->end(); ++i)
//    cerr << *i << endl;
    
  return true;
}

private:
//bool
//get_traj_fval(const std::string& planstr,RawInput* raw_in)
//{
//  std::cerr << "get_traj_fval()..." << std::endl;

//  trajectory_msgs::JointTrajectory traj;
//  traj = utils::get_plan(planstr);

//  if(traj.points.empty())
//  {
//    ROS_WARN("traj.points.empty()");
//    return false;
//  }
//  
//  // For the final robot configuration, note the the start configuration is obtained in get_geo_fval() using srcstate
//  for(size_t i=0; i<traj.joint_names.size(); ++i)
//  {
//    std::string label = std::string("qg."+traj.joint_names.at(i));
//    double value = traj.points.back().positions.at(i);
//    
//    raw_in->insert( std::make_pair(label,value) );
//  }
//  
//  // Extract trajectory representation
//  std::vector<size_t> rep_idxes;
//  if( !get_traj_rep(traj,&rep_idxes) )
//    return false;
//  
//  for(std::vector<size_t>::const_iterator i=rep_idxes.begin(); i!=rep_idxes.end(); ++i)
//  {
//    size_t ith = i-rep_idxes.begin();
//    
//    for(size_t j=0; j<traj.joint_names.size(); ++j)
//    {
//      std::string label = std::string("qw"+boost::lexical_cast<std::string>(ith+1)+"."+traj.joint_names.at(j));
//      double value = traj.points.at(*i).positions.at(j);
//      
//      raw_in->insert( std::make_pair(label,value) );
//    }
//  }
//  
//  return true;
//}

////! Obtain a representation of a trajectory
///*!
//Method: 
//Given a number of desired waypoint, g_np, to represent a trajectory,
//this return indexes of g_np waypoint, excluding the start and the final robot configurations.
//*/
//bool
//get_traj_rep(const trajectory_msgs::JointTrajectory& traj,std::vector<size_t>* chosen_idxes)
//{
//  std::cerr << "get_traj_rep()..." << std::endl;
//  std::cerr << "traj.points.size()= " << traj.points.size() << std::endl;
//  
//  size_t np = g_np;  
////  if(np > (traj.points.size()-2))// minus qs and qg points
////    np = (traj.points.size()-2);
//  if(np > (traj.points.size()-2))// minus qs and qg points
//  {
//    std::cerr << "lack of waypoints" << std::endl;
//    return false;
//  }
//  
//  std::set<size_t> chosen_idxes_set;
//  for(size_t i=0; i<g_np; ++i)
//  {
//    size_t idx;
//    bool passed = false;
//    do
//    {
//      boost::uniform_int<> uni_int_dist( 1,traj.points.size()-1-1 );// range = [a,b]
//      idx = uni_int_dist(rng_);
//      
//      std::pair<std::set<size_t>::iterator,bool> ret;
//      ret = chosen_idxes_set.insert(idx);
//      
//      passed = ret.second;
//    }
//    while(!passed);
//    
//    chosen_idxes->push_back(idx);
//  }
//  std::sort(chosen_idxes->begin(),chosen_idxes->end());// the order is preserved: ascending
//  
////  std::cerr << "chosen_idxes= ";
////  for(size_t i=0; i<chosen_idxes.size(); ++i)
////    std::cerr << chosen_idxes.at(i) << " ";
////  std::cerr << std::endl;
//  
//  return true;
//}

////! Keeps input-only data 
//InputOnlyData data_;
////! For random number generator
//boost::mt19937 rng_;
//! ...
std::string metadata_path_;
};

}// namespace dc_prj6

#endif // #ifndef DATA_COLLECTOR_PRJ6_HPP_INCLUDED
