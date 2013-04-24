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

#include "ml_util.hpp"
#include <lwpr.hh>

static boost::mt19937 gen( std::time(0) );

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
sense(const size_t& n_movable_object)
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
    
    req.bool_args.push_back(false);// rerun
    req.bool_args.push_back(true);// randomized_vase_pose
    
    req.uint_args.push_back(n_movable_object);
    req.uint_args.push_back(1);// for n_vase
    
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
  For MLMode refer to ml_util.hpp
*/
bool
plan(const MLMode& ml_mode,const bool rerun=false,const std::string& log_path=std::string(""),std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol=0,std::vector<double>* ctamp_log=0)
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
    ROS_ERROR("Call to planner_manager/plan srv: FAILED");
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
  

  std::vector< std::list<std::string> > db;// 1st Dimension -> instance type; 2nd Dimension -> sources path
  
  std::list<std::string> a_srcpaths;
  a_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.3/1obj");
  a_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.4/1obj");
  db.push_back(a_srcpaths);
  
  std::list<std::string> b_srcpaths;
  b_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.3/2obj");
  b_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.4/2obj");
  db.push_back(b_srcpaths);
  
  std::list<std::string> c_srcpaths;
  c_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.3/3obj");
  c_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.4/3obj");
  c_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.5/3obj");
  db.push_back(c_srcpaths);
  
  std::list<std::string> d_srcpaths;
  d_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.3/4obj");
  d_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.4/4obj");
  d_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.5/4obj");
  db.push_back(d_srcpaths);
  
  std::list<std::string> e_srcpaths;
  e_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.3/5obj");
  e_srcpaths.push_back("/home/vektor/rss-2013/data/baseline/v.5/5obj");
  db.push_back(e_srcpaths);
  
  // Retrieving
  for(std::vector< std::list<std::string> >::const_iterator i=db.begin(); i!=db.end(); ++i)
  {
    bool specific;
    specific = n_obj;// n_obj = 0 means NOT-specific
  
    size_t instance_type;
    instance_type = i - db.begin() + 1; // +1 because the vector idx begins at 0 and instance_type = [1,5]
      
    if(specific and (instance_type!=n_obj)) continue;
    
    for(std::list<std::string>::const_iterator j=i->begin(); j!=i->end(); ++j)
    {
      boost::filesystem::path dir_path(*j);
      if ( !exists(dir_path) ) continue;
      
      boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
      for ( boost::filesystem::directory_iterator itr( dir_path );itr != end_itr; ++itr )
      {
        if ( is_directory(itr->status()) )
          instance_paths->push_back( itr->path().string() );
      }
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
        
        boost::filesystem::copy_file( std::string(base_data_path+tidy_cfg_filename),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
        
        // Sense!
        gm.sense(n_obj);
        
        // Plan, rerun=false
        MLMode mode = NO_ML;
        bool rerun = false;
        std::string log_path;// not used in this mode
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go
        
        if( !gm.plan(mode,rerun,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << j+1  );
          break;
        }
        
        if( !ctamp_sol.empty() )
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
//    case 7:// SENSE-PLAN(UCS) with a test-bed messy config under the base_data_path one time only
//    {
//      std::string data_path;
//      data_path = base_data_path + suffix_data_path;
//      
//      ros::param::set("/data_path",data_path);
//      boost::filesystem::create_directories(data_path);
//      
//      // Sense
//      gm.sense(std::string(base_data_path+messy_cfg_filename));
//      
//      // Plan, rerun=false
//      MLMode ml_mode;
//      ml_mode = NO_ML;
//      
//      gm.plan(ml_mode);// mode=1 -> UCS, no learning 
//      
//      break;
//    }
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
    case 10:
    // Run online LWPR that updates its model during search; Do runs on multiple instances.
    // USAGE: $ roslaunch hiro_common a.launch  mode:=10 n_obj:=3 runth:=1
    {
      // Init + Collect instances
      std::string run_id;
      run_id = "/h.onlwpr." + boost::lexical_cast<string>(n_obj) + "obj.run" + boost::lexical_cast<string>(runth);
      
      std::vector<std::string> instance_paths;
      set_instance_paths(n_obj,&instance_paths);
      if( instance_paths.empty() ) return false;
      
      std::string ml_data_path;
      ml_data_path = "/home/vektor/rss-2013/data/ml_data/h.onlwpr.log";
      
      std::ofstream run_log;
      run_log.open(std::string(ml_data_path+run_id+".log").c_str());
      
      // Randomize instances, important because the data order matters as it influences how the model is updated; 
      // works like randperm() in matlab
      boost::uniform_int<> dist( 0,instance_paths.size()-1 ) ;
      boost::variate_generator< boost::mt19937&, boost::uniform_int<> > rnd(gen,dist);

      std::vector<size_t> idxes;// Store indexes of the randomized instance order
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
      std::string log_path = std::string(ml_data_path+run_id);// in particular, for ml_data, cost-to-go vs. est. cost-to-go
      boost::filesystem::remove( boost::filesystem::path(log_path+".ml.log") );//remove if exists because will be appended in iteration over instances
      boost::filesystem::remove( boost::filesystem::path(log_path+".h.log") );//remove if exists because will be appended in iteration over instances
        
      std::vector< std::vector<double> > tmp_run_log;
      
      for(std::vector<size_t>::const_iterator i=idxes.begin(); i!=idxes.end(); ++i)
      {
        // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(*i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = run_id;
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
                
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        MLMode mode = LWPR_ONLINE;
        bool rerun = true;
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples at the end of search, (1) # cost-to-go vs. est. cost-to-go

        if( !gm.plan(mode,rerun,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR("gm.plan(mode,rerun,log_path,&ctamp_log): failed");
          break;
        }
        
        tmp_run_log.push_back(ctamp_log);
      }
      
      // Write ctamp log from an ctamp instance
      for(std::vector< std::vector<double> >::const_iterator i=tmp_run_log.begin(); i!=tmp_run_log.end(); ++i)
        run_log << i->at(0) << ",";// write n_samples
      run_log << std::endl;
        
      for(std::vector< std::vector<double> >::const_iterator i=tmp_run_log.begin(); i!=tmp_run_log.end(); ++i)
        run_log << i->at(1) << ",";// write number of cost-to-go vs. est. cost-to-go
      run_log << std::endl;
          
      run_log.close();
      break;
    }
    case 11:
    // Run offline SVR in a batchmode, the model is updated in between search, interleave training and testing; Do runs on multiple instances.
    // USAGE: $ roslaunch hiro_common a.launch  mode:=11 n_obj:=3 runth:=1
    {
      // Init 
      std::string run_id;
      run_id = "/h.offepsvr." + boost::lexical_cast<string>(n_obj) + "obj.run" + boost::lexical_cast<string>(runth);
      
      std::string ml_data_path;
      ml_data_path = "/home/vektor/rss-2013/data/ml_data/h.onlwpr.log";
      
      std::ofstream run_log;
      run_log.open(std::string(ml_data_path+run_id+".log").c_str());
      
      std::string log_path = std::string(ml_data_path+run_id);// in particular, for ml_data, cost-to-go vs. est. cost-to-go
      boost::filesystem::remove( boost::filesystem::path(log_path+".ml.log") );//remove if exists because will be appended in iteration over instances
      boost::filesystem::remove( boost::filesystem::path(log_path+".h.log") );//remove if exists because will be appended in iteration over instances
      
      for(boost::filesystem::directory_iterator itr( boost::filesystem::path("/home/vektor/hiroken-ros-pkg/learning_machine/data/hot") ),end_itr; itr != end_itr; ++itr) remove(itr->path());// delete any file under ML's hot dir
      
      // Collect instances + Randomize instances, 
      // important because the data order matters as it influences how the model is updated; training is interleaved in between testing (search)
      // works like randperm() in matlab
      std::vector<std::string> instance_paths;
      set_instance_paths(n_obj,&instance_paths);
      if( instance_paths.empty() ) return false;
      
      boost::uniform_int<> dist( 0,instance_paths.size()-1 ) ;
      boost::variate_generator< boost::mt19937&, boost::uniform_int<> > rnd(gen,dist);

      std::vector<size_t> idxes;// Store indexes of the randomized instance order
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
      
      // Run the for several instances
      for(std::vector<size_t>::const_iterator i=idxes.begin(); i!=idxes.end() and ros::ok(); ++i)
      {
       // Prepare dir + tidy.cfg file
        base_data_path = instance_paths.at(*i);
        ros::param::set("/base_data_path",base_data_path);
        
        suffix_data_path = run_id;
        ros::param::set("/suffix_data_path",suffix_data_path);
        
        std::string data_path;
        data_path = base_data_path + suffix_data_path;
        ros::param::set("/data_path",data_path);
        
        boost::filesystem::create_directories(data_path);
        
        boost::filesystem::copy_file( std::string(base_data_path+"/tidy.cfg"),std::string(data_path+"/tidy.cfg"),boost::filesystem::copy_option::overwrite_if_exists );
                
        // Sense
        gm.sense( std::string(base_data_path+messy_cfg_filename) );
        
        // Plan 
        bool rerun = true;
        MLMode mode; !(i-idxes.begin()) ? mode = NO_ML : mode = SVR_OFFLINE;// for the first fed intance, NO_ML is trained yet.
        std::vector<trajectory_msgs::JointTrajectory> ctamp_sol;
        std::vector<double> ctamp_log;// Keep data from an CTAMP attempts: (0)n_samples, (1) number of (cost-to-go vs. est. cost-to-go)
        
        if( !gm.plan(mode,rerun,log_path,&ctamp_sol,&ctamp_log) )// Informed search, with the (planned) TMM under base_path
        {
          ROS_ERROR_STREAM( "gm.plan(...): failed on runth=" << i-idxes.begin()+1 << "... " << instance_paths.at(*i) );
          break;
        }
      }
      
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
