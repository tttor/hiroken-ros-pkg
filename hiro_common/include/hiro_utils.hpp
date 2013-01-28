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

std::string
get_eof_link(const std::string& rbt_id)
{
  std::map<std::string,std::string> rbtid_eoflink_map;
  
  if(!set_rbtid_eoflink_map(&rbtid_eoflink_map))
  {
    std::cerr << "Can not rbtid_eoflink_map" << std::endl;
    return std::string("");
  }
  
  return rbtid_eoflink_map[rbt_id];
}

#endif // #ifndef HIRO_UTILS_HPP_INCLUDED
