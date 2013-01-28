#ifndef OBJ_CFG_RW_HPP_INCLUDED
#define OBJ_CFG_RW_HPP_INCLUDED

#include "obj_cfg_rw.hpp"

#include <arm_navigation_msgs/CollisionObject.h>

#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string/split.hpp>

// Three kinds of tabletop objects that we consider, 
// i.e. A (Georgia coffee blue), B (Suntory white), C (Gebisu brown)
static const double A_RADIUS = 0.052/2.;
static const double A_HEIGHT = 0.105;// This height seems too low
static const double B_RADIUS = 0.065/2.;
static const double B_HEIGHT = 0.123;
static const double C_RADIUS = 0.065/2.;
static const double C_HEIGHT = 0.167;

typedef std::vector<arm_navigation_msgs::CollisionObject> ObjCfg;

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
        object.header.frame_id = "/table";
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
   cerr << "Unable to open the foo.cfg file" << endl;
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

#endif // #ifndef OBJ_CFG_RW_HPP_INCLUDED
