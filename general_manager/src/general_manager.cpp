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
#include <ros/package.h>
#include <arm_navigation_msgs/RobotTrajectory.h>

#include <boost/filesystem.hpp>
#include <fstream>

#include "sensor_manager/Sense.h"
#include "planner_manager/Plan.h"
#include "planner_manager/Misc.h"
#include "action_manager/Commit.h"
#include "action_manager/Go2Home.h"
#include "learning_machine/Train.h"

#include <algorithm>    // std::next_permutation, std::sort
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include "ml_util.hpp"
#include "utils.hpp"
#include <lwpr.hh>

using namespace std;

class GeneralManager
{
public:
GeneralManager(ros::NodeHandle& nh)
  :nh_(nh)
{
  online_ = false;// Set the default
  if( !ros::param::get("/online", online_) )
    ROS_WARN("Can not get /online, use the default value (=false) instead");
    
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

//! Sense with a built messy cfgs
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
  
  req.bool_args.push_back(true);// rerun=yes
  req.bool_args.push_back(false);// consequently,randomized_vase=false;
  
  req.uint_args.push_back(size_t());//any
  req.uint_args.push_back(size_t());//any
  
  req.string_args.push_back(path);

  if( !sense_client.call(req,res) ) 
  {
    ROS_ERROR("Call to /sense srv: FAILED");
    return false;
  }
  
  return true;
}

//! sense with expected n_movable_object
bool
sense(const size_t& n_movable_object,const bool& randomized_vase=false,const size_t& n_vase=1)
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
    
    req.bool_args.push_back(false);// 0: rerun
    req.bool_args.push_back(randomized_vase);// 1
    
    req.uint_args.push_back(n_movable_object);//0
    req.uint_args.push_back(n_vase);//1
    
    req.string_args.push_back(std::string());
    
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
  For ml_util::MLMode refer to ml_util.hpp
*/
bool
plan(const ml_util::MLMode& ml_mode,const bool rerun=false,const std::string& ml_hot_path=std::string(""),const std::string& log_path=std::string(""),std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol=0,std::vector<double>* ctamp_log=0)
{
  ros::service::waitForService("/plan");
    
  ros::ServiceClient plan_client;
  plan_client = nh_.serviceClient<planner_manager::Plan> ("/plan");
  
  planner_manager::Plan::Request req;
  planner_manager::Plan::Response res;
  
  req.ml_mode = ml_mode;
  req.rerun = rerun;  
  req.ml_hot_path = ml_hot_path;
  req.log_path = log_path;
  
  if( !plan_client.call(req, res) ) 
  {
    ROS_ERROR("Call to /plan srv: FAILED");
    return false;
  }

  if(ctamp_sol != NULL)
    *ctamp_sol = res.ctamp_sol;
      
  if(ctamp_log != NULL)
    *ctamp_log = res.ctamp_log;
  
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
    ROS_WARN("Call to /commit srv: FAILED");
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
    ROS_WARN("Call to /train srv: FAILED");
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

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    
  GeneralManager gm(nh);
  
  int mode = 0;
  if( !ros::param::get("/mode",mode) )
    ROS_WARN("Can not get /mode, use the default value (=0) instead");
  cerr << "mode= " << mode << endl;
  
  int n_obj = 0;
  if( !ros::param::get("/n_obj",n_obj) )
    ROS_WARN("Can not get /n_obj, use the default value (=0) instead");

  int n_run = 1;
  if( !ros::param::get("/n_run", n_run) )
    ROS_WARN("Can not get /n_run, use the default value instead"); 
    
  int epsth = 1;
  if( !ros::param::get("/epsth", epsth) )
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

  std::vector<std::string> mode1011_instance_paths(n_run);// used in mode1011,mode10,mode11
  
  switch(mode)
  {
    case 1:
    // SENSE-PLAN with zeroed-H, solve CTAMP by seaching over TMM using UCS, with randomized messy_cfg, n times
    // The result is stored under "path" whose default value is set in the launch file
    // USAGE: $ roslaunch hiro_common a.launch mode:=1 n_obj:=1 n_run:=2 tidy_cfg:=/tidy_baseline.1.cfg suffix:=20130430a
    {
      for(size_t j=0; j<(size_t)n_run; ++j)
      {
        std::string data_path;
        data_path = base_data_path + "/run." + boost::lexical_cast<std::string>(n_obj) + "obj." + suffix_data_path + "." +  boost::lexical_cast<std::string>(j);
        ros::param::set("/data_path",data_path);
        
        // Directory for this CTAMP attempt
        boost::filesystem::create_directories(data_path);
        
        // Copy a tidy_cfg from the one under base_path
        boost::filesystem::copy_file( std::string(base_data_path+tidy_cfg_filename),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        
        // Sense!
        bool randomized_vase = false;
        size_t n_vase = 1;
        gm.sense(n_obj,randomized_vase,n_vase);
        
        // Plan, rerun=false
        ml_util::MLMode mode = ml_util::NO_ML;
        bool rerun = false;
        std::string log_path;// not used in this mode
        std::string ml_hot_path;// not used in this mode
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go
        
        if( !gm.plan(mode,rerun,ml_hot_path,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on epsth=" << j+1  );
          break;
        }
        
        // Remove all files if this CTAMP attempt returns no solution because for now we focus only on the one who has a solution if seach with UCS
        if( ctamp_sol.empty() )
          boost::filesystem::remove_all( boost::filesystem::path(data_path) );
        else
          utils::create_makepngsh(base_data_path,data_path);
      }

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
    case 7:
    // SENSE-PLAN(UCS) with a test-bed messy config under the base_data_path one time only, aimed for debugging
    // $ roslaunch hiro_common a.launch mode:=7 path:=? suffix:=?
    {
      messy_cfg_filename = "/messy.cfg";
      tidy_cfg_filename = "/tidy.cfg";
      
      std::string data_path;
      data_path = base_data_path + suffix_data_path;
      
      ros::param::set("/data_path",data_path);
      boost::filesystem::create_directories(data_path);
      boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
      
      // Sense
      gm.sense(std::string(base_data_path+messy_cfg_filename));
      
      // Plan, rerun=false
      ml_util::MLMode mode = ml_util::NO_ML;
      bool rerun = false;
      std::string log_path;// not used in this mode
      std::string ml_hot_path;// not used in this mode
      std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
      std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go
      
      if( !gm.plan(mode,rerun,ml_hot_path,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
      {
        ROS_ERROR("gm.plan(...): failed");
      }
      
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
    case 1011:
      // Do mode10 and mode 11 subsequently with the same instance_paths order
      // USAGE: $ roslaunch hiro_common a.launch  mode:=1011 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline n_obj:=1 n_run:=10 epsth:=1 
      if( !utils::get_instance_paths(boost::filesystem::path(base_data_path),std::string(boost::lexical_cast<std::string>(n_obj)+"obj"),&mode1011_instance_paths) )
      {
        ROS_ERROR("utils::get_instance_paths() -> failed");
        return false;
      }
      
      // No break is necessary!
      
    case 10:
    // Run online LWPR that updates its model during search; Do one CTAMP attempt (rerun) on multiple instances.
    // WARNING: DO NOT run multiple mode10 simultaneously, critical resources are at /learning_machine/data/hot
    // Number of attempts is determined in the inst_paths file
    // USAGE: $ roslaunch hiro_common a.launch  mode:=10 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline n_obj:=1 epsth:=1 n_run:=2
    // \param path holds the parent directory under which CTAMP instances exist 
    // \param n_obj holds the instance type
    // \param epsth holds i-th episode with this mode
    {
      // Init
      std::string run_id;
      run_id = "/h.onlwpr." + boost::lexical_cast<string>(n_obj) + "M" + "." + boost::lexical_cast<string>(epsth);
      
      std::vector<std::string> instance_paths;
      if(mode == 1011)
      {
        instance_paths = mode1011_instance_paths;
      }
      else
      {
        std::string inst_path_filename;
        inst_path_filename = std::string("inst_paths."+boost::lexical_cast<std::string>(n_obj)+"M."+boost::lexical_cast<std::string>(epsth));
        
        if( !utils::get_instance_paths(std::string(base_data_path+"/"+inst_path_filename),&instance_paths) )
        {
          ROS_ERROR("utils::get_instance_paths() -> failed");
          return false;
        }
        
        // Synch. with the requested n_run
        if(n_run > instance_paths.size())
          ROS_WARN("n_run > instance_paths.size(), use all instance_paths.size()");
        else
        {
          std::vector<std::string> tmp_inst_paths(instance_paths.begin(),instance_paths.begin()+n_run);
          instance_paths = tmp_inst_paths;
        }
      }
      
      std::string ml_pkg_path = ros::package::getPath("learning_machine");
      std::string ml_hot_path = std::string(ml_pkg_path+"/data/hot"+run_id);

      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      boost::filesystem::create_directories(ml_hot_path);
           
      // Run mode10 for several CTAMP instances
      std::string log_dir_path = std::string("/home/vektor/rss-2013/data/with_v.4.3/mode10eps.log");
      if( !boost::filesystem::exists(boost::filesystem::path(log_dir_path)) )
        boost::filesystem::create_directories(log_dir_path);
        
      std::string log_path = std::string(log_dir_path+run_id);
      boost::filesystem::remove( boost::filesystem::path(log_path+".ml.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".h.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".h2.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".log") );
        
      std::vector< std::pair< std::string,std::vector<double> > > mode10eps_log;
      
      for(int i=0; i<instance_paths.size(); ++i)
      {
        ROS_DEBUG_STREAM( "On attemp th=" << i+1 << "... " << instance_paths.at(i) << " on mode= " << mode );
  
        // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = std::string(run_id+"."+boost::lexical_cast<std::string>(i+1));
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::remove_all( boost::filesystem::path(data_path) );
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        boost::filesystem::copy_file( std::string(base_data_path+"/vanilla_tmm.dot"),std::string(data_path+"/vanilla_tmm.dot"),boost::filesystem::copy_option::overwrite_if_exists );// because this is rerun so that no call to the task plan
                
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        ml_util::MLMode ml_mode = ml_util::LWPR_ONLINE;
        bool rerun = true;
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go (2) # h_v,h_av,c

        if( !gm.plan(ml_mode,rerun,ml_hot_path,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << i+1 << "... " << instance_paths.at(i) << " on mode= " << mode );
          return false;
        }
        
        mode10eps_log.push_back( std::make_pair(base_data_path,ctamp_log) );
        utils::create_makepngsh(base_data_path,data_path);
      }
      
      // Closure
      utils::write_log(mode10eps_log,std::string(log_path+".log"));
      
      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      
      if(mode != 1011) 
      {
        break;
      }
      else// If mode==1011,  clear n_ctamp_attempt_ of planner_manager node then jump to mode11
      {
        ros::service::waitForService("/clear_n_ctamp_attempt");
    
        ros::ServiceClient clearing_client;
        clearing_client = nh.serviceClient<planner_manager::Misc> ("/clear_n_ctamp_attempt");
        
        planner_manager::Misc::Request req;
        planner_manager::Misc::Response res;
        
        if( !clearing_client.call(req,res) ) 
        {
          ROS_ERROR("Call to /clear_n_ctamp_attempt srv: FAILED");
          return false;
        }
      }
    }
    case 11:
    // Run offline SVR in a batchmode, the model is updated in between search, interleave training and testing; Do runs on multiple instances.
    // Number of attempts is determined in the inst_paths file
    // WARNING: DO NOT run multiple mode11 simultaneously, critical resources are at /learning_machine/data/hot
    // USAGE:$ roslaunch hiro_common a.launch  mode:=11 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline n_obj:=1 epsth:=1 n_run:=2
    {
      // Init  
      std::string run_id;
      run_id = "/h.offepsvr." + boost::lexical_cast<string>(n_obj) + "M"+ "." + boost::lexical_cast<string>(epsth);
      
      std::vector<std::string> instance_paths;
      if(mode == 1011)
      {
        instance_paths = mode1011_instance_paths;
      }
      else
      {
        std::string inst_path_filename;
        inst_path_filename = std::string("inst_paths."+boost::lexical_cast<std::string>(n_obj)+"M."+boost::lexical_cast<std::string>(epsth));
        
        if( !utils::get_instance_paths(std::string(base_data_path+"/"+inst_path_filename),&instance_paths) )
        {
          ROS_ERROR("utils::get_instance_paths() -> failed");
          return false;
        }
        
        // Synch. with the requested n_run
        if(n_run > instance_paths.size())
          ROS_WARN("n_run > instance_paths.size(), use all instance_paths.size()");
        else
        {
          std::vector<std::string> tmp_inst_paths(instance_paths.begin(),instance_paths.begin()+n_run);
          instance_paths = tmp_inst_paths;
        }
      }
      
      std::string ml_pkg_path = ros::package::getPath("learning_machine");
      std::string ml_hot_path = std::string(ml_pkg_path+"/data/hot"+run_id);

      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      boost::filesystem::create_directories(ml_hot_path);

      // Run mode11 for several instances
      std::string log_dir_path = std::string("/home/vektor/rss-2013/data/with_v.4.3/mode11eps.log");
      if( !boost::filesystem::exists(boost::filesystem::path(log_dir_path)) )
        boost::filesystem::create_directories(log_dir_path);
      
      std::string log_path = std::string(log_dir_path+run_id);
      boost::filesystem::remove( boost::filesystem::path(log_path+".ml.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".h.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".h2.log") );
      boost::filesystem::remove( boost::filesystem::path(log_path+".log") );
      
      std::vector< std::pair< std::string,std::vector<double> > > mode11eps_log;
      
      for(int i=0; i<instance_paths.size(); ++i)
      {
        ROS_DEBUG_STREAM( "On attempt th=" << i+1 << "... " << instance_paths.at(i) << " on mode= " << mode );
        
        // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = std::string(run_id+"."+boost::lexical_cast<std::string>(i+1));
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::remove_all( boost::filesystem::path(data_path) );
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        boost::filesystem::copy_file( std::string(base_data_path+"/vanilla_tmm.dot"),std::string(data_path+"/vanilla_tmm.dot"),boost::filesystem::copy_option::overwrite_if_exists );// because this is rerun so that no call to the task plan
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        bool rerun = true;
        ml_util::MLMode ml_mode = ml_util::SVR_OFFLINE;
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples, (1) number of (cost-to-go vs. est. cost-to-go) (2) # h_v,h_av,c
        
        if( !gm.plan(ml_mode,rerun,ml_hot_path,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << i+1 << "... " << instance_paths.at(i) << " on mode= " << mode );
          return false;
        }

        mode11eps_log.push_back( std::make_pair(base_data_path,ctamp_log) );
        utils::create_makepngsh(base_data_path,data_path);
      }// for each instance
      
      // Closure
      utils::write_log(mode11eps_log,std::string(log_path+".log"));
      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      
      break;
    }
    case 13:
    // For collecting path samples offline from UCS-planned TMM
    // Usage: $ roslaunch hiro_common a.launch  mode:=13 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline.tr n_obj:=1 n_run:=10
    {
      // Init  
      std::string run_id;
      run_id = "/h.zeroed." + boost::lexical_cast<string>(n_obj) + "M";
      
      std::vector<std::string> instance_paths(n_run);
      if( !utils::get_instance_paths(boost::filesystem::path(base_data_path),std::string(boost::lexical_cast<std::string>(n_obj)+"obj"),&instance_paths) )
      {
        ROS_ERROR("utils::get_instance_paths() -> failed");
        return false;
      }
      
      std::string ml_pkg_path = ros::package::getPath("learning_machine");
      std::string ml_hot_path = std::string(ml_pkg_path+"/data/hot"+run_id);

      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      boost::filesystem::create_directories(ml_hot_path);
      
      // Run mode13 for several instances
      for(int i=0; i<n_run; ++i)
      {
        // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = std::string(run_id);
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::remove_all( boost::filesystem::path(data_path) );
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        boost::filesystem::copy_file( std::string(base_data_path+"/vanilla_tmm.dot"),std::string(data_path+"/vanilla_tmm.dot"),boost::filesystem::copy_option::overwrite_if_exists );// because this is rerun so that no call to the task plan
        
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        bool rerun = true;
        ml_util::MLMode ml_mode = ml_util::NO_ML_BUT_COLLECTING_SAMPLES;
        
        if( !gm.plan(ml_mode,rerun,ml_hot_path) )
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << i+1 << "... " << instance_paths.at(i) << "on mode= " << mode );
          return false;
        }

        utils::create_makepngsh(base_data_path,data_path);
      }// for each instance
      
      // Copy data from data from ml hot path to ml offline path then remove the hot ones
      std::string ml_offline_data_dir = "/home/vektor/rss-2013/data/with_v.4.3/ml_offline_data/";
      
      boost::filesystem::copy_file( std::string(ml_hot_path+"/tr_data.libsvmdata"),std::string(ml_offline_data_dir + "tr_data." + boost::lexical_cast<string>(n_obj) + "M" + ".libsvmdata"),boost::filesystem::copy_option::overwrite_if_exists );// for tuning ml
      boost::filesystem::copy_file( std::string(ml_hot_path+"/tr_data.csv"),std::string(ml_offline_data_dir + "tr_data." + boost::lexical_cast<string>(n_obj) + "M" +".csv"),boost::filesystem::copy_option::overwrite_if_exists );// for tuning ml
      
      boost::filesystem::remove_all( boost::filesystem::path(ml_hot_path) );
      
      break;
    }
    case 14:
    // For writing instance order into a file, should be rarely used
    // Usage: $ roslaunch hiro_common a.launch  mode:=14 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline
    {
    
      size_t n_eps = 10;
      
      std::vector<size_t> n_instance_per_instance_type(6);
      n_instance_per_instance_type.at(0) = 100;
      n_instance_per_instance_type.at(1) = 600;
      n_instance_per_instance_type.at(2) = 300;
      n_instance_per_instance_type.at(3) = 150;
      n_instance_per_instance_type.at(4) = 75;
      n_instance_per_instance_type.at(5) = 35;
      
      for(size_t i=0; i < n_eps; ++i)
      {
        for(size_t j=0; j < 6; ++j)// iterating over 6 instance types
        {
          size_t n_obj = j;
          size_t n_run = n_instance_per_instance_type.at(j);
          
          std::vector<std::string> instance_paths(n_run);
          
          // Get instance order
          utils::get_instance_paths(boost::filesystem::path(base_data_path),std::string(boost::lexical_cast<std::string>(n_obj)+"obj"),&instance_paths);
            
          // Write          
          utils::write_instance_path(instance_paths,base_data_path,n_obj,i+1);
        }
      }
      
      // Init  
      std::string run_id;
      run_id = "/h.zeroed." + boost::lexical_cast<string>(n_obj) + "M";
      
      std::vector<std::string> instance_paths(n_run);
      if( !utils::get_instance_paths(boost::filesystem::path(base_data_path),std::string(boost::lexical_cast<std::string>(n_obj)+"obj"),&instance_paths) )
      {
        ROS_ERROR("utils::get_instance_paths() -> failed");
        return false;
      }
    
      break;
    }
    case 15:
    // For rebase-ing i.e. fix some stuff in the baseline
    // Assume that al directories under path(base_data_path) are baseline directories
    // This overwrites all perf logs, tmm.dot
    // Usage: $ roslaunch hiro_common a.launch  mode:=15 path:=/home/vektor/rss-2013/data/with_v.4.3/baseline.rebased.test
    {
      // Get all directory under path, assume all directories are the baselines
      std::vector<std::string> instance_paths;
      
      boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
      for ( boost::filesystem::directory_iterator itr(base_data_path);itr != end_itr; ++itr )
      {
        if ( is_directory(itr->status()) )
          instance_paths.push_back( itr->path().string() );
      }
      
      // Run for all instances
      for(size_t i=0; i<instance_paths.size(); ++i)
      {
        ROS_DEBUG_STREAM("Rebasing " << i+1 << " of " << instance_paths.size());

        // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(i);
        ros::param::set("/base_data_path",base_data_path);
        
        std::string data_path;
        data_path = base_data_path;
        ros::param::set("/data_path",data_path);

        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );

        // Plan, rerun=false
        ml_util::MLMode mode = ml_util::NO_ML;
        bool rerun = true;
        std::string log_path;// not used in this mode
        std::string ml_hot_path;// not used in this mode
        
        if( !gm.plan(mode,rerun,ml_hot_path) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << i+1 << "... " << instance_paths.at(i) << " on mode= " << mode );
          break;
        }
      }
      break;
    }
    case 12:
    // For prj-6. Run pick-and-place with 1 movable object with one arm with one joint-space set
    // USAGE: $ roslaunch hiro_common prj-6.launch n_run:=? suffix:=?
    {
      n_obj = 1;
      
      for(size_t j=0; j<(size_t)n_run; ++j)
      {
        std::string data_path;
        data_path = base_data_path + suffix_data_path + "." +  boost::lexical_cast<std::string>(j);
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+tidy_cfg_filename),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        
        // Sense!
        bool randomized_vase = true;
        size_t n_vase = 2;
        gm.sense(n_obj,randomized_vase,n_vase);
        
        // Plan, rerun=false
        ml_util::MLMode mode = ml_util::NO_ML;
        bool rerun = false;
        std::string log_path;// not used in this mode
        std::string ml_hot_path;// not used in this mode
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go
        
        if( !gm.plan(mode,rerun,ml_hot_path,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on epsth=" << j+1  );
          break;
        }
        
        if( ctamp_sol.empty() )
         boost::filesystem::remove_all( boost::filesystem::path(data_path) );
      }// for each run
 
      break;
    }
    default:
    {
      ROS_WARN("Unknown mode; do nothing!");
    }
  }// end of: switch(mode)
  
  ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> general_mgr: spinning... <<<<<<<<");
  ros::spin();
  
  return 0;
}
