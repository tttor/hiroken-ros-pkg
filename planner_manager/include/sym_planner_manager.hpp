#ifndef SYM_PLANNER_MANAGER_HPP_INCLUDED
#define SYM_PLANNER_MANAGER_HPP_INCLUDED

#include <ros/ros.h>

#include "task_planner/PlanTask.h"

#include "planner_manager.hpp"
#include "tmm_utils.hpp"

class SymbolicPlannerManager
{
public:
SymbolicPlannerManager(const ros::NodeHandle& nh)
: nh_(nh)
{ }

~SymbolicPlannerManager()
{ }

bool
plan(PlannerManager pm)
{
  // Make a request to plan_task srv
  task_planner::PlanTask::Request plan_task_req;
  task_planner::PlanTask::Response plan_task_res;
  
  for(std::map<std::string, arm_navigation_msgs::CollisionObject>::const_iterator i=pm.movable_obj_messy_cfg_.begin(); i!=pm.movable_obj_messy_cfg_.end(); ++i)
    plan_task_req.objects.push_back(i->second);
  
  ros::service::waitForService("plan_task"); 
  ros::ServiceClient plan_task_client;
  plan_task_client = nh_.serviceClient<task_planner::PlanTask>("plan_task");  

  if( plan_task_client.call(plan_task_req, plan_task_res) )
  {
    ROS_DEBUG("plan_task service call: OK");
  }
  else
  {
    ROS_ERROR("plan_task service call: NOT OK");
    return false;
  }
  
  return true;
}

private:
ros::NodeHandle nh_;
//! A helper variable
/*!
  More ...
*/
};

#endif // #ifndef SYM_PLANNER_MANAGER_HPP_INCLUDED
