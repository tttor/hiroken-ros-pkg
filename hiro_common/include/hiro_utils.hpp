#ifndef HIRO_UTILS_HPP_INCLUDED
#define HIRO_UTILS_HPP_INCLUDED

#include "hiro_utils.hpp"

bool
set_rbtid_eoflink_map(std::map<std::string,std::string>* map_ptr)
{
  // Hardcoded setting here!
  (*map_ptr)["RARM"] = std::string("link_rhand_palm");
  (*map_ptr)["LARM"] = std::string("link_lhand_palm");
  
  return true;
}

//! "rbtid" means robot's effectors, e.g. arm
bool
set_rbtid_biggestjspace_map(std::map<std::string,std::string>* map_ptr)
{
  // Hardcoded setting here!
  (*map_ptr)["RARM"] = std::string("rarm_U_chest");
  (*map_ptr)["LARM"] = std::string("larm_U_chest");
  
  return true;
}

bool
set_jspace_baselink_map(std::map<std::string,std::string>* map_ptr)
{
  // Hardcoded setting here! MUST be consistent with hiro_planning_description.yaml
  // TODO access directly from hiro_planning_description.yaml
  (*map_ptr)["rarm"] = std::string("link_chest_yaw");
  (*map_ptr)["rarm_U_chest"] = std::string("link_base");
  (*map_ptr)["larm"] = std::string("link_chest_yaw");
  (*map_ptr)["larm_U_chest"] = std::string("link_base");
    
  return true;
}

bool
set_jspace_tiplink_map(std::map<std::string,std::string>* map_ptr)
{
  // Hardcoded setting here! MUST be consistent with hiro_planning_description.yaml
  // TODO access directly from hiro_planning_description.yaml
  (*map_ptr)["rarm"] = std::string("link_rhand_palm");
  (*map_ptr)["rarm_U_chest"] = std::string("link_rhand_palm");
  (*map_ptr)["larm"] = std::string("link_lhand_palm");
  (*map_ptr)["larm_U_chest"] = std::string("link_lhand_palm");
    
  return true;
}

std::map<std::string,std::string>
get_rbtid_biggestjspace_map()
{
  std::map<std::string,std::string> map;
  
  set_rbtid_biggestjspace_map(&map);
  
  return map;
}

std::string
get_eof_link(const std::string& rbt_id)
{
  std::map<std::string,std::string> map;
  
  if(!set_rbtid_eoflink_map(&map))
  {
    std::cerr << "Can not set the map" << std::endl;
    return std::string("");
  }
  
  return map[rbt_id];
}

std::string
get_base_link(const std::string& jspace)
{
  std::map<std::string,std::string> map;
  
  if(!set_jspace_baselink_map(&map))
  {
    std::cerr << "Can not set the map" << std::endl;
    return std::string("");
  }
  
  return map[jspace];
}

std::string
get_tip_link(const std::string& jspace)
{
  std::map<std::string,std::string> map;
  
  if(!set_jspace_tiplink_map(&map))
  {
    std::cerr << "Can not set the map" << std::endl;
    return std::string("");
  }
  
  return map[jspace];
}

size_t
get_jspace_size(const std::string& jspace)
{
  if( !strcmp(jspace.c_str(),std::string("rarm").c_str()) or !strcmp(jspace.c_str(),std::string("larm").c_str()) )
    return 6;
  else if( !strcmp(jspace.c_str(),std::string("rarm_U_chest").c_str()) or !strcmp(jspace.c_str(),std::string("larm_U_chest").c_str()) )
    return 7;
  else
    return std::numeric_limits<size_t>::max();
}

#endif // #ifndef HIRO_UTILS_HPP_INCLUDED
