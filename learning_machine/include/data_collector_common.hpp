#ifndef DATA_COLLECTOR_COMMON_HPP_INCLUDED
#define DATA_COLLECTOR_COMMON_HPP_INCLUDED

#include "data_collector_common.hpp"
#include "data.hpp"
#include "tmm_utils.hpp"

namespace dc_common
{
//! Obtain geometric-feature values from srcstate, which is an edge property
/*!
  Geometric features include:
  (1) object pose in the source vertex
  
  
*/
bool 
get_geo_fval(const std::string& srcstate,RawInput* r_in,const std::string& suffix="")
{
  std::cerr << "get_geo_fval()..." << std::endl;

  // Init 
  std::vector<std::string> srcstate_parts;
  boost::split( srcstate_parts, srcstate, boost::is_any_of(";") );
  
  if( srcstate_parts.at(0).size() == 0 )
  {
    cerr << "srcstate_parts.at(0).size() == 0" << endl;
    return false;
  }

  std::map<std::string,double> jname_jpos_map;// for obtaining jstate later on
  
  //(1) object pose in the source-vertex's state
  for(std::vector<std::string>::const_iterator i=srcstate_parts.begin(); i!=srcstate_parts.end(); ++i )
  {
    std::vector<std::string> comps;
    boost::split( comps, *i, boost::is_any_of(",") );
    
    if(comps.size()==8)// object's pose data: id, x, y, z, qx, qy, qz, qw
    {
      std::string obj_id = comps.at(0);
      
      comps.erase(comps.begin());// remove the id so as to make comps and names (below) exactly have 7 elements indicating the pose
      
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

  //(2) joint-state (robot configuration) on the source vertex, jname_jpos_map is set in step (1)
  for(std::map<std::string,double>::const_iterator i=jname_jpos_map.begin(); i!=jname_jpos_map.end(); ++i)
  {
    std::string label = std::string("qs."+i->first);
    double value = i->second;
    
    r_in->insert(  std::make_pair( label,value )  );
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

}// namespace dc_common

#endif // #ifndef DATA_COLLECTOR_PRJ6_HPP_INCLUDED
