#ifndef UTILS_HPP_INCLUDED
#define UTILS_HPP_INCLUDED

#include "utils.hpp"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <arm_navigation_msgs/CollisionObject.h>

#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

namespace utils
{
// Three kinds of tabletop objects that we consider, 
// i.e. A (Georgia coffee blue), B (Suntory white), C (Gebisu brown)
static const double A_RADIUS = 0.052/2.;
static const double A_HEIGHT = 0.105;// This height seems too low
static const double B_RADIUS = 0.065/2.;
static const double B_HEIGHT = 0.123;
static const double C_RADIUS = 0.065/2.;
static const double C_HEIGHT = 0.167;

static boost::mt19937 g_rng( std::time(0) );

typedef std::vector<arm_navigation_msgs::CollisionObject> ObjCfg;

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

bool
read_obj_cfg(const std::string& cfg_path,ObjCfg* cfg)
{
  using namespace std;
  
  ifstream cfg_in(cfg_path.c_str());
  if(cfg_in.is_open())
  {
    string line;
    
    arm_navigation_msgs::CollisionObject object;
    arm_navigation_msgs::Shape object_shape;
    geometry_msgs::Pose object_pose;
  
    while ( cfg_in.good() )
    {
      getline(cfg_in,line);
//      cout << line << endl;
      
      if( !strcmp(line.c_str(),"===") )
      {
        object.shapes.push_back(object_shape);
        object.poses.push_back(object_pose);

        cfg->push_back(object);
        
        // reset
        object.shapes.clear();
        object.poses.clear();
        
        continue;
      }

      vector<string> line_comps;
      boost::split( line_comps, line, boost::is_any_of("=") );
      
      if( !strcmp(line_comps.at(0).c_str(),"id") )
      {
        object.id = line_comps.at(1);
        object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
      }
      else if( !strcmp(line_comps.at(0).c_str(),"frame") )
      {
        object.header.frame_id = line_comps.at(1);
      }
      else if( !strcmp(line_comps.at(0).c_str(),"pos") )
      {
        vector<string> pos_comps;
        boost::split( pos_comps, line_comps.at(1), boost::is_any_of(",") );
        
        object_pose.position.x = boost::lexical_cast<double>( pos_comps.at(0) );
        object_pose.position.y = boost::lexical_cast<double>( pos_comps.at(1) );
        object_pose.position.z = boost::lexical_cast<double>( pos_comps.at(2) );
      }
      else if( !strcmp(line_comps.at(0).c_str(),"ori") )
      {
        vector<string> ori_comps;
        boost::split( ori_comps, line_comps.at(1), boost::is_any_of(",") );
        
        object_pose.orientation.x = boost::lexical_cast<double>( ori_comps.at(0) );
        object_pose.orientation.y = boost::lexical_cast<double>( ori_comps.at(1) );
        object_pose.orientation.z = boost::lexical_cast<double>( ori_comps.at(2) );
        object_pose.orientation.w = boost::lexical_cast<double>( ori_comps.at(3) );
      }
      else if( !strcmp(line_comps.at(0).c_str(),"dim") )
      {
        vector<string> dim_comps;
        boost::split( dim_comps, line_comps.at(1), boost::is_any_of(",") );
        
        if(dim_comps.size()==2)
        {
          object_shape.type = arm_navigation_msgs::Shape::CYLINDER;// TODO make it flexible if necessary
          object_shape.dimensions.resize(dim_comps.size());
          object_shape.dimensions[0] = boost::lexical_cast<double>( dim_comps.at(0) );// cyl radius
          object_shape.dimensions[1] = boost::lexical_cast<double>( dim_comps.at(1) );// cyl height
        }
      }
    }

    cfg_in.close();
  }
  else
  {
   cerr << "Unable to open" << cfg_path << endl;
   return false;
  }
  
//  for(ObjCfg::const_iterator i=cfg->begin(); i!=cfg->end(); ++i)
//  {
//    cerr << "id=" << i->id << endl;
////    cerr << "x,y,z= " << i->poses.at(0).pos ...
//  }
  
  return true;
}

bool
write_obj_cfg(const ObjCfg& cfg,const std::string& path)
{
  using namespace std;
  
  ofstream cfg_out(path.c_str());
  
  if (cfg_out.is_open())
  {
    for(ObjCfg::const_iterator i=cfg.begin(); i!=cfg.end(); ++i)
    {
      cfg_out << "id=" << i->id << endl;
      
      cfg_out << "frame=" << i->header.frame_id << endl;
      
      cfg_out << "pos=" << i->poses.at(0).position.x << "," << i->poses.at(0).position.y << "," << i->poses.at(0).position.z << endl;
      cfg_out << "ori=" << i->poses.at(0).orientation.x << "," << i->poses.at(0).orientation.y << "," << i->poses.at(0).orientation.z << "," << i->poses.at(0).orientation.w << endl;
      
      cfg_out << "dim=" << i->shapes.at(0).dimensions.at(0) << "," << i->shapes.at(0).dimensions.at(1) << endl;
      
      cfg_out  << "===" << endl;
    }
    
    cfg_out.close();
    return true;
  }
  else
  {
   cerr << "Unable to open file to be written.";
   return false;
  }
}

void
print_robot_state(const sensor_msgs::JointState& js)
{
  using namespace std;
  
  for(size_t i=0; i<js.name.size(); ++i)
  {
    cout << js.name.at(i) << "= " << js.position.at(i) << endl;
  }
}

void
print_robot_state(const trajectory_msgs::JointTrajectory& trj,const size_t& ith_point)
{
  using namespace std;
  
  for(size_t i=0; i<trj.joint_names.size(); ++i)
  {
    cout << trj.joint_names.at(i) << "= " << trj.points.at(ith_point).positions.at(i) << endl;
  }
}

//! Obtain selected instances as _TEST_ beds
bool
get_instance_paths(const boost::filesystem::path& path,const std::string& inst_type,std::vector<std::string>* inst_paths)
{
  using namespace std;
  
  if( !exists(path) or !is_directory(path) )
  {
    cerr << "!exists(path) or !is_directory(path)" << endl;
    return false;
  }

  bool specific;
  if( !strcmp(inst_type.c_str(),std::string("0obj").c_str()) )
    specific = false;
  else
    specific = true;
  
  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for ( boost::filesystem::directory_iterator itr(path);itr != end_itr; ++itr )
  {
    if ( is_directory(itr->status()) )
    {
      std::string inst_path;
      inst_path = itr->path().string();
      
      if(!specific)
      {
        inst_paths->push_back(inst_path);
      }
      else
      {
        std::vector<std::string> inst_path_parts;
        boost::split( inst_path_parts,inst_path,boost::is_any_of("/") );// Split e.g. /home/vektor/rss-2013/data/with_v.4.2/baseline/run.1obj.20130430a.1
        
        std::vector<std::string> inst_path_subparts;
        boost::split( inst_path_subparts,inst_path_parts.back(),boost::is_any_of(".") );
        if(inst_path_subparts.at(0).size() == 0) return false;
        
        std::string local_inst_type;
        local_inst_type = inst_path_subparts.at(1);
        
        if( !strcmp(local_inst_type.c_str(),inst_type.c_str()) )
          inst_paths->push_back(inst_path);
      }
    }
  }

  return true;
}

//! Randomize instances, important because the data order matters as it influences how the model is updated; works like randperm() in matlab
void
randomize(std::vector<std::string>* inst_paths)
{
  if (inst_paths->empty() )
    return;

  boost::uniform_int<> dist( 0,inst_paths->size()-1 ) ;
  boost::variate_generator< boost::mt19937&, boost::uniform_int<> > rnd(g_rng,dist);

  std::vector<size_t> idxes;// Store indexes of the randomized instance order
  for(size_t i=0; i<inst_paths->size(); ++i)
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
  
  std::vector<std::string> tmp_inst_paths;
  tmp_inst_paths = *inst_paths;
  for(size_t i=0; i<tmp_inst_paths.size(); ++i)
    inst_paths->at(i) = tmp_inst_paths.at(idxes.at(i));
}

void
create_makepngsh(const std::string& dir_path)
{
  std::ofstream sh;
  sh.open(std::string(dir_path+"/make_png.sh").c_str());
  
  sh << "dot -Tpng tmm.dot -o tmm.png" << std::endl;
  sh << "dot -Tpng vanilla_tmm.dot -o vanilla_tmm.png" << std::endl;  
  sh << "dot -Tpng fancy_tmm.dot -o fancy_tmm.png" << std::endl;
  sh << "dot -Tpng task_graph.dot -o task_graph.png" << std::endl;
  sh << "dot -Tpng sol_tmm.dot -o sol_tmm.png" << std::endl;
  
  sh.close();
}

void
create_makepngsh(const std::string& src_dir_path,const std::string& dir_path)
{
  boost::filesystem::copy_file( std::string(src_dir_path+"/make_png.sh"),std::string(dir_path+"/make_png.sh"),boost::filesystem::copy_option::overwrite_if_exists );  
}

void
write_log(const std::vector< std::pair< std::string,std::vector<double> > >& log,const std::string& path)
{
  std::ofstream log_out;
  log_out.open(path.c_str());
  
  for(size_t i=0; i<log.size(); ++i)
  {
    log_out << log.at(i).first;
    if(i < (log.size()-1)) log_out << ",";
  }
  log_out << std::endl;
              
  for(size_t i=0; i<log.size(); ++i)
  {
    log_out << log.at(i).second.at(0);// write n_samples
    if(i < (log.size()-1)) log_out << ",";
  }
  log_out << std::endl;
    
  for(size_t i=0; i<log.size(); ++i)
  {
    log_out << log.at(i).second.at(1);// write number of pairs of (cost-to-go vs. est. cost-to-go), also indicates the depth of solution
    if(i < (log.size()-1)) log_out << ",";
  }
  log_out << std::endl;
      
  log_out.close();
}

}// namespace utils

#endif // #ifndef UTILS_HPP_INCLUDED
