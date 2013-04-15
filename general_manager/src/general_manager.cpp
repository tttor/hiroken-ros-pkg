/*
MODES
case 1:// SENSE-PLAN with zeroed-H, solve CTAMP by seaching over TMM using UCS, with randomized messy_cfg, n times
  $ roslaunch hiro_common a.launch mode:=1 n_obj:=1 n_run:=1 tidy_cfg:=/tidy_baseline.1.cfg suffix:=/run.test.20130220 

case 2:// Execute the resulted CTAMP 
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

#include <lwpr.hh>
#include "ml_util.hpp"

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
plan(const MLMode& ml_mode,const bool rerun=false,const std::string& log_path=std::string(""),uint32_t* n_samples=0)
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request req;
  planner_manager::Plan::Response res;
  
  req.ml_mode = ml_mode;
  req.log_path = log_path;
  req.rerun = rerun;
  
  if( !plan_client.call(req, res) ) 
  {
    ROS_WARN("Call to planner_manager/plan srv: FAILED");
    return false;
  }
  
  *n_samples = res.n_samples;
  
  return !( res.ctamp_sol.empty() );
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
set_instance_paths(const size_t n_obj,std::vector<std::string>* instance_paths)
{
  std::vector< std::list<std::string> > db;
  
  // Type: 1-obj instances
  std::list<std::string> a_paths;
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.0");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.1");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.2");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.3");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130308a.4");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.0");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.1");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.2");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.3");
  a_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj/run.1obj.20130310a.4");// 10 
  
  db.push_back(a_paths);
  
  // Type: 2-obj instances
  std::list<std::string> b_paths;
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.0");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.1");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.2");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308a.4");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308b.0");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130308b.3");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.0");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.1");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130310a.2");
  b_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj/run.2obj.20130313a.0");// 10
  
  db.push_back(b_paths);
  
  // Type: 3-obj instances
  std::list<std::string> c_paths;
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.0");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.2");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.3");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.5");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.6");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130307b.7");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310a.0");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310d.2");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130310f.0");
  c_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj/run.3obj.20130312f.4");// 10  
  
  db.push_back(c_paths);
  
  // Type: 4-obj instances
  std::list<std::string> d_paths;
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.1");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.4");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.7");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.8");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130307.9");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130308c.0");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130308c.2");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130309.3");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130310b.0");
  d_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj/run.4obj.20130310h.0");// 10
  
  db.push_back(d_paths);
  
  // Type: 5-obj instances
  std::list<std::string> e_paths;  
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.0");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.2");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310a.3");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310b.0");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130310c.0");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130312k.0");
  e_paths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj/run.5obj.20130312o.0");// 7
  
  db.push_back(e_paths);
  
  // Retrieving
  if(n_obj==0)// alternate run, then put all type-instances together
  {
    for(std::vector< std::list<std::string> >::const_iterator i=db.begin(); i!=db.end(); ++i)
    {
      for(std::list<std::string>::const_iterator j=i->begin(); j!=i->end(); ++j)
      {
        instance_paths->push_back(*j);
      }
    }
  }
  else// only n_obj-object instaces
  {
    for(std::list<std::string>::const_iterator j=db.at(n_obj-1).begin(); j!=db.at(n_obj-1).end(); ++j)
    {
      instance_paths->push_back(*j);
    }
  }
 
  ROS_INFO_STREAM("instance_paths->size()= " << instance_paths->size());
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
    
  int runth = 1;
  if( !ros::param::get("/runth", runth) )
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
        
        // Sense!
        gm.sense(n_obj);
        
        // Plan, rerun=false
        MLMode ml_mode;
        ml_mode = NO_ML;
        
        if( gm.plan(ml_mode) )// mode=1 -> UCS, no learning 
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
//    case 3:// TRAIN the ML offline (data are already available)
//    {
//      std::vector<std::string> tmm_paths;
//      tmm_paths.push_back(data_path+"/tmm.dot");
//      
//      gm.train(tmm_paths);
//      break;
//    }
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
      
      // Sense
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      
      // Plan, rerun=false
      MLMode ml_mode;
      ml_mode = NO_ML;
      
      gm.plan(ml_mode);// mode=1 -> UCS, no learning 
      
      break;
    }
//    case 8:// For testing (single run only) the heuristic machine::SVRegress that is trained offline and does not do incremental online learning.
//    {
//      std::string data_path;
//      data_path = base_data_path + suffix_data_path;
//      
//      ros::param::set("/data_path",data_path);
//      boost::filesystem::create_directories(data_path);
//      
//      gm.sense(std::string(base_data_path+messy_cfg_filename));
//      
//      size_t plan_mode;
//      plan_mode = 2;// Offline SVR
//      
//      gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
//      
//      break;
//    }
//    case 9:// Use LWPR offline one batch, do NOT update its model online (during search)
//    {
//      suffix_data_path = "/h.lwpr.off";
//      ros::param::set("/suffix_data_path",suffix_data_path);
//        
//      std::string data_path;
//      data_path = base_data_path + suffix_data_path;
//      ros::param::set("/data_path",data_path);
//      
//      boost::filesystem::create_directories(data_path);
//      
//      gm.sense(std::string(base_data_path+messy_cfg_filename));
//      
//      size_t plan_mode;
//      plan_mode = 3;
//      
//      gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
//      
//      break;
//    }
    case 10:// Run online LWPR that updates its model during search on multiple instances.USAGE: $ roslaunch hiro_common a.launch  mode:=10 n_obj:=3 runth:=1
    {
      // Init + Collect instances
      std::string run_id;
      run_id = "/h.onlwpr." + boost::lexical_cast<string>(n_obj) + "obj.run" + boost::lexical_cast<string>(runth);
      
      std::vector<std::string> instance_paths;
      set_instance_paths(n_obj,&instance_paths);
      
      std::string ml_data_path;
      ml_data_path = "/home/vektor/rss-2013/data/ml_data/h.onlwpr.log";
      
      std::ofstream run_log;
      run_log.open(std::string(ml_data_path+run_id+".log").c_str());
      
      // Randomize instances, important because the data order matters as it influences how the model is updated; like randperm() in matlab
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
        run_log << instance_paths.at(idx) << ",";
      }
      run_log << std::endl;
      
      // Initialize the vanilla LWPR model
      size_t input_dim = 68;
      size_t output_dim = 1;
      LWPR_Object lwpr(input_dim,output_dim);
      
      lwpr.setInitD(0.05);/* Set initial distance metric to D*(identity matrix) */
      lwpr.updateD(true);// Determines whether distance matrix updates are to be performed
      lwpr.setInitAlpha(0.1);/* Set init_alpha to _alpha_ in all elements */
      lwpr.useMeta(true);// Determines whether 2nd order distance matrix updates are to be performed

      std::string lwpr_model_path;
      lwpr_model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/lwpr.bin";
      
      if( !lwpr.writeBinary(lwpr_model_path.c_str()) )
      {
        ROS_ERROR("lwpr.writeBinary()= FAILED");
        return false;
      }
           
      // Run the trials
      for(std::vector<size_t>::const_iterator i=idxes.begin(); i!=idxes.end(); ++i)
      {
        // Prepare dir
        base_data_path = instance_paths.at(*i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = run_id;
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
                
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        MLMode mode;
        mode = LWPR_ONLINE;
        
        bool rerun;
        rerun = true;
        
        std::string log_path;
        log_path = std::string(ml_data_path+run_id);
        
        uint32_t n_samples;// Store n_samples at the end of planning for this instance
        
        gm.plan(mode,rerun,log_path,&n_samples);// Informed search, with the (planned) TMM under base_path

        run_log << n_samples << ",";
      }
      
      run_log.close();
      break;
    }
//    case 11:// For testing the heuristic machine::SVRegress that is trained offline and does not do incremental online learning, multiple runs
//    {
//      // Collect instances
//      std::vector<std::string> instance_paths;
//      set_instance_paths(1,&instance_paths);
//      
//      // Execute
//      for(std::vector<std::string>::const_iterator i=instance_paths.begin(); i!=instance_paths.end(); ++i)
//      {
//        // Prepare dir
//        base_data_path = *i;
//        ros::param::set("/base_data_path",base_data_path);
//        
//        suffix_data_path = "/h.epsvr";
//        ros::param::set("/suffix_data_path",suffix_data_path);
//        
//        std::string data_path;
//        data_path = base_data_path + suffix_data_path;
//        ros::param::set("/data_path",data_path);
//        
//        boost::filesystem::create_directories(data_path);
//        
//        // Sense
//        gm.sense( std::string(base_data_path+messy_cfg_filename) );
//        
//        // Plan 
//        MLMode ml_mode;
//        ml_mode = SVR_OFFLINE;
//      
//        bool rerun;
//        rerun = true;
//        
//        gm.plan(ml_mode,rerun);// Informed search, with the (planned) TMM under base_path
//      }
//      
//      break;
//    }
//    case 12:
//    {
//      // Collect instances
//      std::vector<std::string> instance_paths;
//      set_instance_paths(&instance_paths);
//      
//      // Execute
//      for(std::vector<std::string>::const_iterator i=instance_paths.begin(); i!=instance_paths.end(); ++i)
//      {
//        // Prepare dir
//        base_data_path = *i;
//        ros::param::set("/base_data_path",base_data_path);
//        
//        suffix_data_path = "/h.offlwpr";
//        ros::param::set("/suffix_data_path",suffix_data_path);
//        
//        std::string data_path;
//        data_path = base_data_path + suffix_data_path;
//        ros::param::set("/data_path",data_path);
//        
//        boost::filesystem::create_directories(data_path);
//        
//        // Sense
//        gm.sense( std::string(base_data_path+messy_cfg_filename) );
//        
//        // Plan 
//        size_t plan_mode;
//        plan_mode = 3;// Offline LWPR
//      
//        gm.plan(plan_mode);// Informed search, with the (planned) TMM under base_path
//      }
//      
//      break;
//    }
    default:
    {
      ROS_WARN("Unknown mode; do nothing!");
    }
  }// end of: switch(mode)
  
  ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> general_mgr: spinning... <<<<<<<<");
  ros::spin();
  
  return 0;
}
