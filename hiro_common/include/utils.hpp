#ifndef UTILS_HPP_INCLUDED
#define UTILS_HPP_INCLUDED

#include "utils.hpp"
#include <boost/algorithm/string.hpp>
#include <trajectory_msgs/JointTrajectory.h>

trajectory_msgs::JointTrajectory
get_plan(const std::string& planstr)
{
  trajectory_msgs::JointTrajectory plan;  
  
  plan.header.stamp = ros::Time::now();
  
  std::vector<std::string> planstr_subs;
  boost::split( planstr_subs, planstr, boost::is_any_of(";") );
  
  boost::split( plan.joint_names, planstr_subs.at(0), boost::is_any_of(",") );  
  planstr_subs.erase( planstr_subs.begin() );
  
  for(std::vector<std::string>::const_iterator i=planstr_subs.begin(); i!=planstr_subs.end(); ++i)
  {
    std::vector<std::string> positions_str;
    boost::split( positions_str, *i, boost::is_any_of(",") );
    
    std::vector<double> positions;
    for(std::vector<std::string>::const_iterator j=positions_str.begin(); j!=positions_str.end(); ++j)
      positions.push_back( boost::lexical_cast<double>(*j) );
      
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    
    plan.points.push_back(point);
  }
  
  return plan;  
}

#endif // #ifndef UTILS_HPP_INCLUDED
