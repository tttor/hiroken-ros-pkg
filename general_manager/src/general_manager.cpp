/*
MODES
case 1:// SENSE-PLAN with zeroed-H, solve CTAMP by seaching over TMM using UCS, with randomized messy_cfg, n times
  $ roslaunch hiro_common a.launch mode:=1 n_obj:=1 n_run:=1 tidy_cfg:=/tidy_baseline.1.cfg suffix:=/run.test.20130220 

case 2:
  $roslaunch hiro_common a.launch mode:=2 path:=/home/vektor/rss-2013/data/run.2obj.20130220.0
  
case 4:// randomized-SENSE only, messy.cfg is written in the base_data_path
  $ roslaunch hiro_common a.launch mode:=4 n_obj:=5
  
case 5:// SENSE TIDY-cfg only, assume that the cfg file is under base_data_path
  $ roslaunch hiro_common a.launch mode:=5 tidy_cfg:=/tidy_tb2.l.cfg

case 6:// messy_tb-SENSE only, assume that the cfg file is under base_data_path
  $ roslaunch hiro_common a.launch mode:=6 messy_cfg:=/messy_hot.cfg
  
case 7:// SENSE-PLAN(UCS) with a test-bed messy config under the base_data_path one time only
  $ roslaunch hiro_common a.launch mode:=7 messy_cfg:=/messy.cfg tidy_cfg:=/tidy_baseline.1.cfg suffix:=/run.test.20130220

case 8:
  $ roslaunch hiro_common a.launch mode:=8 path:=/home/vektor/rss-2013/data/baseline/v.2/2obj/run.2obj.20130220.0 suffix:=/h_test-1

case 9:
  $ roslaunch hiro_common a.launch  mode:=9 path:=/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.1

case 10:
  $ roslaunch hiro_common a.launch  mode:=10
*/
#include <ros/ros.h>
#include <arm_navigation_msgs/RobotTrajectory.h>

#include <boost/filesystem.hpp>
#include <fstream>

#include "sensor_manager/Sense.h"
#include "planner_manager/Plan.h"
#include "action_manager/Commit.h"
#include "action_manager/Go2Home.h"
#include "learning_machine/Train.h"

#include <algorithm>    // std::next_permutation, std::sort
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

static boost::mt19937 gen( std::time(0) );

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

//! Sense with test-bed messy cfgs
bool
sense(const std::string& path)
{
  // NOTE: may need at least 3 times (?), otherwise the planner_manager will miss collission object publications.
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
  
  return true;
}

bool
sense(const size_t& n)
{
//  // At least 3 times, otherwise the planner_manager will miss collission object publications.
//  for(size_t i=0; i<3; ++i)
//  {
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
//  }
  
  return true;
}

//! Brief
/*!
  Mode = 
  1 (without heuristic learner, heuristic is always equal to zero, UCS)
  2 (with heuristic learner, the learning machine is trained offline, use epsilon-svr)
  3 (with heuristic learner: LWPR, do NOT update its model online (during search)
  4 (with heuristic learner: LWPR and train incrementally online)
*/
bool
plan(const size_t& mode)
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request req;
  planner_manager::Plan::Response res;
  
  req.mode = mode;
  
  if( !plan_client.call(req, res) ) 
  {
    ROS_WARN("Call to planner_manager/plan srv: FAILED");
    return false;
  }
  
  return !(res.ctamp_sol.empty());
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

//! All selected instances as _TEST_ beds are here!
void
set_instance_paths(std::vector<std::string>* instance_paths)
{
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.1");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.3");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.4");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.1");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.3");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.4");// 10
  
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.1");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.4");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308b.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308b.3");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.1");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130313a.0");// 10

  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.0");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.2");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.3");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.5");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.6");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.7");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310a.0");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310d.2");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310f.0");
  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130312f.4");// 10
  
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.1");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.4");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.7");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.8");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.9");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130308c.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130308c.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130309.3");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130310b.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130310h.0");// 10
  
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.2");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.3");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310b.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310c.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130312k.0");
//  instance_paths->push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130312o.0");// 7
  
  cerr << "instance_paths->size()= " << instance_paths->size() << endl;
}

int 
factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

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

  int n_run = 1;
  if( !ros::param::get("/n_run", n_run) )
    ROS_WARN("Can not get /n_run, use the default value instead"); 
    
  std::string base_data_path;// The base_data_path is a constant parameter-server
  if( !ros::param::get("/base_data_path", base_data_path) )
    ROS_WARN("Can not get /base_data_path, use the default value instead");  

  std::string suffix_data_path;
  if( !ros::param::get("/suffix_data_path", suffix_data_path) )
    ROS_WARN("Can not get /suffix_data_path, use the default value instead"); 

  std::string tidy_cfg_filename;
  if( !ros::param::get("/tidy_cfg_filename",tidy_cfg_filename) )
    ROS_WARN("Can not get /tidy_cfg_filename, use the default value instead"); 
  
  std::string messy_cfg_filename;
  if( !ros::param::get("/messy_cfg_filename",messy_cfg_filename) )
    ROS_WARN("Can not get /messy_cfg_filename, use the default value instead"); 

  switch(mode)
  {
    case 1:// SENSE-PLAN with zeroed-H, solve CTAMP by seaching over TMM using UCS, with randomized messy_cfg, n times
    {
      // Open a log file for this episode
      std::ofstream epi_log;
      epi_log.open(std::string(base_data_path+suffix_data_path+".log").c_str());
      
      epi_log << "n_run= " << n_run << endl;
      epi_log << "successful_run_idx= ";
      for(size_t j=0; j<(size_t)n_run; ++j)
      {
        std::string data_path;
        data_path = base_data_path + suffix_data_path + "." +  boost::lexical_cast<std::string>(j);
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        gm.sense(n_obj);
        
        if( gm.plan(1) )// mode=1 -> UCS, no learning 
          epi_log << j << ",";
      }
      epi_log << endl;
      
      epi_log.close();
      break;
    }
    case 2:// SENSE-ACT with any existing manipulation plan in the base_data_path
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
    case 4:// randomized-SENSE only, messy.cfg is written in the base_data_path
    {
      gm.sense(n_obj);
      break;
    }
    case 5:// SENSE TIDY-cfg only, assume that the cfg file is under base_data_path
    {
      gm.sense(std::string(base_data_path+tidy_cfg_filename));
      break;
    }
    case 6:// messy_tb-SENSE only, assume that the cfg file is under base_data_path
    {
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      break;
    }
    case 7:// SENSE-PLAN(UCS) with a test-bed messy config under the base_data_path one time only
    {
      std::string data_path;
      data_path = base_data_path + suffix_data_path;
      
      ros::param::set("/data_path",data_path);
      boost::filesystem::create_directories(data_path);
      
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      
      // Plan
      gm.plan(1);// mode=1 -> UCS, no learning 
      
      break;
    }
    case 8:// For testing (single run only) the heuristic machine::SVRegress that is trained offline and does not do incremental online learning.
    {
      std::string data_path;
      data_path = base_data_path + suffix_data_path;
      
      ros::param::set("/data_path",data_path);
      boost::filesystem::create_directories(data_path);
      
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      
      size_t plan_mode;
      plan_mode = 2;// Offline SVR
      
      gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
      
      break;
    }
    case 9:// Use LWPR offline, do NOT update its model online (during search)
    {
      suffix_data_path = "/h.lwpr.off";
      ros::param::set("/suffix_data_path",suffix_data_path);
        
      std::string data_path;
      data_path = base_data_path + suffix_data_path;
      ros::param::set("/data_path",data_path);
      
      boost::filesystem::create_directories(data_path);
      
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      
      size_t plan_mode;
      plan_mode = 3;
      
      gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
      
      break;
    }
    case 10:// Run LWPR that updates its model during search on multiple instances
    {
      // Collect instances
      std::vector<std::string> instance_paths;
      set_instance_paths(&instance_paths);
      
      // Randomize instances, important because the data order matters as it influences how the model is updated; like randperm()
      boost::uniform_int<> dist( 0,instance_paths.size()-1 ) ;
      boost::variate_generator< boost::mt19937&, boost::uniform_int<> > rnd(gen,dist);

      std::vector<size_t> idxes;
      for(size_t i=0; i<instance_paths.size(); ++i)
      {
        size_t idx;
        bool already;
        
        do
        {
          already = false;
          
          idx = rnd();
          
          std::vector<size_t>::iterator it;
          it = std::find(idxes.begin(),idxes.end(),idx);
          
          if(it != idxes.end())
            already = true;
        }
        while(already);
        
        idxes.push_back(idx);
      }
      
      // Execute
      std::ofstream ith_log;
      ith_log.open("/home/vektor/rss-2013/data/onlwpr_log/onlwpr_3obj_3.log");

      std::ofstream ith_log_2;
      ith_log_2.open("/home/vektor/rss-2013/data/onlwpr_log/onlwprbl_3obj_3.log");
      
      for(std::vector<size_t>::const_iterator i=idxes.begin(); i!=idxes.end(); ++i)
      {
        // Prepare dir
        base_data_path = instance_paths.at(*i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = "/h.onlwpr_3obj_3";
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        ith_log << "data_onlwpr_3obj_3 = [data_onlwpr_3obj_3; csvread('" << data_path << "/perf.log.csv')];" << endl;
        ith_log_2 << "data_onlwprbl_3obj_3 = [data_onlwprbl_3obj_3; csvread('" << base_data_path << "/perf.log.csv')];" << endl;
                
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        size_t plan_mode;
        plan_mode = 4;// Online LWPR
      
        gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
      }
      
      ith_log.close();
      ith_log_2.close();
      
      break;
    }
    case 11:// For testing the heuristic machine::SVRegress that is trained offline and does not do incremental online learning, multiple runs
    {
      // Collect instances
      std::vector<std::string> instance_paths;
      set_instance_paths(&instance_paths);
      
      // Execute
      for(std::vector<std::string>::const_iterator i=instance_paths.begin(); i!=instance_paths.end(); ++i)
      {
        // Prepare dir
        base_data_path = *i;
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = "/h.epsvr";
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        size_t plan_mode;
        plan_mode = 2;// Offline SVR
      
        gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
      }
      
      break;
    }
    case 12:
    {
      // Collect instances
      std::vector<std::string> instance_paths;
      set_instance_paths(&instance_paths);
      
      // Execute
      for(std::vector<std::string>::const_iterator i=instance_paths.begin(); i!=instance_paths.end(); ++i)
      {
        // Prepare dir
        base_data_path = *i;
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = "/h.offlwpr";
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        size_t plan_mode;
        plan_mode = 3;// Offline LWPR
      
        gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
      }
      
      break;
    }
    default:
    {
      ROS_WARN("Unknown mode; do nothing!");
    }
  }// end of: switch(mode)
  
  ROS_INFO(">>>>>>>> general_mgr: spinning... <<<<<<<<");
  ros::spin();
  
  return 0;
}
