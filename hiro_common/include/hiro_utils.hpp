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

#endif // #ifndef HIRO_UTILS_HPP_INCLUDED
