#include <ros/ros.h>
#include <arm_navigation_msgs/RobotTrajectory.h>

#include <boost/filesystem.hpp>
#include <fstream>

#include "sensor_manager/Sense.h"
#include "planner_manager/Plan.h"
#include "action_manager/Commit.h"
#include "action_manager/Go2Home.h"
#include "learning_machine/Train.h"

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
sense(const std::string& path)
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
    req.uint_args.push_back(false);// randomized=false
    req.uint_args.push_back(0);// any number, meaningless here
    req.string_args.push_back(path);
    
    if( !sense_client.call(req,res) ) 
    {
      ROS_WARN("A call to /sense srv: FAILED");
      return false;
    }
  }
  
  return true;
}

bool
sense(const size_t& n)
{
  // At least 3 times, otherwise the planner_manager will miss collission object publications.
  for(size_t i=0; i<3; ++i)
  {
    ros::service::waitForService("/sense");
      
    ros::ServiceClient sense_client;
    sense_client = nh_.serviceClient<sensor_manager::Sense> ("/sense");
    
    sensor_manager::Sense::Request req;
    sensor_manager::Sense::Response res;
    
    req.id = 1;// for sense::see
    req.uint_args.push_back(true);
    req.uint_args.push_back(n);
    req.string_args.push_back("");
    
    if( !sense_client.call(req,res) ) 
    {
      ROS_WARN("A call to /sense srv: FAILED");
      return false;
    }
  }
  
  return true;
}

//! Brief
/*!
  Mode = 1 (without heuristic learner, heuristic is always equal to zero)
*/
bool
plan(const size_t& mode)
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request req;
  planner_manager::Plan::Response res;
  
  req.mode = 1;
  
  if( !plan_client.call(req, res) ) 
  {
    ROS_WARN("Call to planner_manager/plan srv: FAILED");
    return false;
  }
  
  return !(res.man_plan.empty());
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

bool
train(const std::vector<std::string>& tmm_paths)
{
  ros::service::waitForService("/train");
    
  ros::ServiceClient train_client;
  train_client = nh_.serviceClient<learning_machine::Train> ("/train");
  
  learning_machine::Train::Request req;
  learning_machine::Train::Response res;
  
  req.tmm_paths = tmm_paths;
  
  if( !train_client.call(req, res) ) 
  {
    ROS_WARN("Call to learning_machine/train srv: FAILED");
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
  
  int n_obj = 0;
  if( !ros::param::get("/n_obj",n_obj) )
    ROS_WARN("Can not get /n_obj, use the default value (=0) instead");

  std::string base_data_path;// The base_data_path is a constant
  if( !ros::param::get("/base_data_path", base_data_path) )
    ROS_WARN("Can not get /base_data_path, use the default value instead");  
    
  switch(mode)
  {
    case 1:// SENSE-PLAN with zeroed-H, solve CTAMP by seaching over TMM using UCS.
    {
      std::string suffix_data_path;
      if( !ros::param::get("/suffix_data_path", suffix_data_path) )
        ROS_WARN("Can not get /suffix_data_path, use the default value instead");  
    
      int n_run = 1;
      if( !ros::param::get("/n_run", n_run) )
        ROS_WARN("Can not get /n_run, use the default value instead");  
      
      // Open a log file for this episode
      std::ofstream epi_log;
      epi_log.open(std::string(base_data_path+suffix_data_path+".log").c_str());

      epi_log << "successful_runs= " << endl;
      for(size_t j=0; j<(size_t)n_run; ++j)
      {
        std::string data_path;
        data_path = base_data_path + suffix_data_path + "." +  boost::lexical_cast<std::string>(j);
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        gm.sense(n_obj);
        
        if( gm.plan(1) )// mode=1 -> UCS, no learning 
          epi_log << j << endl;
      }
      
      epi_log.close();
      break;
    }
    case 2:// SENSE-ACT with any existing manipulation plan in the data center
    {
      gm.sense(std::string(base_data_path+"/messy.cfg"));
      gm.act();
      break;
    }
    case 3:// TRAIN the ML offline (data are already available)
    {
//      std::vector<std::string> tmm_paths;
//      tmm_paths.push_back(data_path+"/tmm.dot");
//      
//      gm.train(tmm_paths);
      break;
    }
    case 4:// randomized-SENSE only, messy.cfg is written in base_data_path
    {
      gm.sense(n_obj);
      break;
    }
    case 5:// SENSE TIDY-cfg only
    {
      gm.sense(std::string(base_data_path+"/tidy_tb1.cfg"));
      break;
    }
    default:
    {
      ROS_WARN("Unknown mode; do nothing!");
    }
  }
  
  ROS_INFO("general_mgr: spinning...");
  ros::spin();
  
  return 0;
}