#include <ros/ros.h>
#include <arm_navigation_msgs/RobotTrajectory.h>

#include "sensor_manager/Sense.h"
#include "planner_manager/Plan.h"
#include "action_manager/Commit.h"
#include "action_manager/Go2Home.h"

using namespace std;

class GeneralManager
{
public:
GeneralManager(ros::NodeHandle& nh)
  :nh_(nh)
{
  online_ = false;// Set the default
  if( !ros::param::get("/is_online", online_) )
    ROS_WARN("Can not get /is_online, use the default value (=false) instead");
    
  ros::service::waitForService("/sense");
  ros::service::waitForService("/plan");
  ros::service::waitForService("/move_arm");
  
  ROS_INFO("GeneralManager: UP and RUNNING ...");
  
  if( !online_ )
  {
    ros::service::waitForService("/go2home");
    
    action_manager::Go2Home::Request req;
    action_manager::Go2Home::Response res;
    
    ros::ServiceClient go2home_client;
    go2home_client = nh_.serviceClient<action_manager::Go2Home> ("/go2home");
    
    go2home_client.call(req,res);
  }
}

~GeneralManager()
{ }

bool
sense(const size_t& n, const bool& random)
{
  // At least 3 times, otherwise the planner_manager will miss collission object publications.
  for(size_t i=0; i<3; ++i)
  {
    ros::service::waitForService("/sense");
      
    ros::ServiceClient sense_client;
    sense_client = nh_.serviceClient<sensor_manager::Sense> ("/sense");
    
    sensor_manager::Sense::Request req;
    sensor_manager::Sense::Response res;
    
    req.id = 1;
    req.args.push_back(n);
    req.args.push_back(random);
    
    if( !sense_client.call(req,res) ) 
    {
      ROS_WARN("A call to /sense srv: FAILED");
      return false;
    }
  }
  
  return true;
}

bool
plan()
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request req;
  planner_manager::Plan::Response res;
  
  if( !plan_client.call(req, res) ) 
  {
    ROS_WARN("Call to planner_manager/plan srv: FAILED");
    return false;
  }
  
  man_plan_ = res.man_plan;
  return true;
}

bool
act()
{
  ros::service::waitForService("/commit");
    
  ros::ServiceClient commit_client;
  commit_client = nh_.serviceClient<action_manager::Commit> ("/commit");
  
  action_manager::Commit::Request req;
  action_manager::Commit::Response res;
  
  if( !commit_client.call(req, res) ) 
  {
    ROS_WARN("Call to action_manager/commit srv: FAILED");
    return false;
  }
  
  return true;
}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;
//! Whether this is simulation or a real physical robot.
/*!
  More ...
*/
bool online_;
//! The resulted manipulation plan is stored here
/*!
  It is basically a sequence of Transfer and Transit path along with its geometric (joint_state) level implementation.
*/
std::vector<trajectory_msgs::JointTrajectory> man_plan_;
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "general_mgr");
  ros::NodeHandle nh;
  
  GeneralManager gm(nh);
  
  int mode = 0;
  if( !ros::param::get("/mode",mode) )
    ROS_WARN("Can not get /mode, use the default value (=0) instead");
  
  int n_objs = 0;
  if( !ros::param::get("/n_objs",n_objs) )
    ROS_WARN("Can not get /n_objs, use the default value (=0) instead");
    
  switch(mode)
  {
    case 1:// SENSE-PLAN with zeroed-H
    {
      size_t n = 1;
      for(size_t j=0; j<n; ++j)
      {
        gm.sense(n_objs,false);
        gm.plan();
      }
      break;
    }
    case 2:// SENSE-ACT with any existing manipulation plan in the data center
    {
      gm.sense(n_objs,false);
      gm.act();
      break;
    }
    case 3:
    {
      break;
    }
    default:
      ROS_WARN("Unknown mode; do nothing!");
  }
  
  ROS_INFO("general_mgr: spinning...");
  ros::spin();
  
  return 0;
}
