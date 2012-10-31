#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>

#include "task_planner/PlanTask.h"

struct HighLevelAction
{
  std::string op;
  std::vector<std::string> args;
  std::string w;
  
  std::string
  id()
  {
    std::string id;
    
    id = op;
    for(std::vector<std::string>::const_iterator i=args.begin(); i!=args.end(); ++i)
      id += "_" + *i;
    
    return id;
  }
};

struct Edge
{
  size_t s;
  size_t t;
  
  double w;
  size_t flag;
  
  std::string name;
  
  bool 
  operator==(const Edge& other) const 
  {
    return (this->s==other.s and this->t==other.t);
  }
};
//! A class that does task planning.
/*!
  Ideally, this class has a capability to call CommonLisp-based HTN planner, unfortunately, up to Oct 13, 2012, it does not.
  Therefore, the communication with the planner manager is done via a text file, specified by *.tmg.
  
  To fire up the core process, another process should make a request via a service named plan_task.
  This service returns nothing.
*/
class TaskPlanner
{
public:
TaskPlanner(ros::NodeHandle& nh)
  :nh_(nh)
{
  task_planner_path_ = ".";
  if( !ros::param::get("/task_planner_path", task_planner_path_) )
    ROS_WARN("Can not get /task_planner_path, use the default value instead");
  
  planner_manager_path_ = ".";
  if( !ros::param::get("/planner_manager_path", planner_manager_path_) )
    ROS_WARN("Can not get /task_planner_path, use the default value instead");
    
//  plan();// For testing only!
}

~TaskPlanner()
{
}

//! Brief ...
/*!
  This handles the service named plan_task.
  
  \param req A request: arm_navigation_msgs/CollisionObject[] objects
  \param res A response, which is nothing here
  \return whether successful
*/
bool
plan_task_cb(task_planner::PlanTask::Request& req, task_planner::PlanTask::Response& res)
{
  return plan(req.objects);
}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

std::string task_planner_path_;

std::string planner_manager_path_;

//! Brief ...
/*!
  This is the core function in the TaskPlanner class.
  TODO Calling to HTN planner should be done here.
  
  For now, up to Oct 13, 2012, the communication with a CommonLisp-based HTN plannner is done via a file; hardcoded!
  In the same vein, the commomunication with the planner manager is also done via a file; hardcoded!.
  
  \return whether successful
*/
bool
plan(const std::vector<arm_navigation_msgs::CollisionObject>& objs)
{
  std::string plan_path = task_planner_path_ + "/plans/" ;
  for(size_t i=0; i<objs.size(); ++i)
    plan_path += boost::lexical_cast<std::string>(i+1) + ".";
  plan_path += "plan";
  
  std::ifstream file(plan_path.c_str());

  std::string lines;// all plans in a single string  
  if( file.is_open() )
  {
    while ( file.good() )
    {
      std::string line;
      std::getline(file,line);
      
      lines += " " + line;
      
    }
    file.close();
  }
  else 
  {
    ROS_ERROR("Unable to open a task plan file");
    return false;
  }

  // Remove the head "(" and the tail ")"; the most outer ones
  boost::trim(lines);

  boost::erase_head(lines, 1);
  boost::erase_tail(lines, 1);
  
  // Replacement for an easy splitting; "((" separates plans
  boost::replace_all(lines, "((", "#");
    
  // Parse into plans
  std::vector<std::string> plans;
  boost::split( plans, lines, boost::is_any_of("#") );
  
  // Erase the first element of the vector plans, it contains an empty string, because there is # at the beginning
  plans.erase( plans.begin() );
  
  std::vector<std::string> vertices; vertices.push_back("MessyHome"); vertices.push_back("TidyHome");
  std::vector<Edge>edges;
  
  for(std::vector<std::string>::iterator plan_it=plans.begin(); plan_it!=plans.end(); ++plan_it)
  {
    // Replace "!" with "(!" in each plan AND erase the tail ")"
    boost::trim(*plan_it);
      
    boost::replace_first(*plan_it, "!", "(!");
    boost::erase_tail(*plan_it,1);
    
    // Parsing for plans.tmg
    std::vector<std::string> raw_hlas;
    std::vector<HighLevelAction> hlas;
      
    boost::split( raw_hlas, *plan_it, boost::is_any_of("(!"), boost::algorithm::token_compress_on );
    raw_hlas.erase( raw_hlas.begin() );

    for(std::vector<std::string>::iterator i=raw_hlas.begin(); i!=raw_hlas.end(); ++i)
    {
      boost::erase_all( *i, ")" );
      boost::trim(*i);
      
      std::vector<std::string> hla_parts;
      boost::split( hla_parts, *i, boost::is_any_of(" "), boost::token_compress_on );
      
      HighLevelAction hla;
      hla.op = hla_parts.at(0);
      for(std::vector<std::string>::iterator j=hla_parts.begin()+1; j!=hla_parts.end(); ++j)
      {
        hla.args.push_back(*j);
      }
      hla.args.erase( hla.args.end()-1 );
      
      hla.w = hla_parts.at(hla_parts.size()-1);
      
      hlas.push_back(hla);
    }
    
    std::vector<std::string> released_objects;// For book-keeping which object has been released.
      
    for(std::vector<HighLevelAction>::iterator i=hlas.begin(); i!=hlas.end(); ++i)
    {
      Edge e;
      
      std::string s_str;
      std::string t_str;
      std::vector<std::string>::iterator v_i;
      
      // Infer the source and t vertices
      if( !strcmp(i->op.c_str(), "TRANSIT") )
      {
        if( !strcmp(i->args.at(1).c_str(), "HOME") )
          e.s = 0;
        else
        {
          std::vector<HighLevelAction>::iterator pre_i = i-1;
        
          std::string ps;//  Note that k is incremented up to 
          for(std::vector<std::string>::const_iterator k=released_objects.begin(); k!=(released_objects.end()-2); ++k)
            ps += *k;
            
          s_str = "Released_" + pre_i->args.at(1) + "[" + ps + "]";
          
          v_i = std::find(vertices.begin(), vertices.end(), s_str);
          
          e.s = v_i - vertices.begin();
        }

        if( !strcmp(i->args.at(2).c_str(), "HOME") )
          t_str = "TidyHome";
        else
        {
          std::vector<HighLevelAction>::iterator post_i = i+1;
          
          std::string ps;
          for(std::vector<std::string>::const_iterator k=released_objects.begin(); k!=released_objects.end(); ++k)
            ps += *k;
          
          t_str = "CanGrasp_" + post_i->args.at(1) + "[" + ps + "]";
        }
        
        v_i = std::find(vertices.begin(), vertices.end(), t_str);

        if(v_i==vertices.end())
        {
          vertices.push_back(t_str);

          e.t = vertices.size()-1;
        }
        else
          e.t = v_i - vertices.begin();
      }
      else if( !strcmp(i->op.c_str(), "TRANSFER") )
      {
        std::string ps;
        for(std::vector<std::string>::const_iterator k=released_objects.begin(); k!=released_objects.end(); ++k)
          ps += *k;
      
        s_str = "Grasped_" + i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), s_str);
        
        e.s = v_i - vertices.begin();
        
        std::vector<HighLevelAction>::iterator post_i = i+1;
        
        t_str = "CanRelease_" + post_i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), t_str);

        if(v_i==vertices.end())
        {
          vertices.push_back(t_str);

          e.t = vertices.size()-1;
        }
        else
          e.t = v_i - vertices.begin();
      }
      else if( !strcmp(i->op.c_str(), "GRASP") )
      {
        std::string ps;
        for(std::vector<std::string>::const_iterator k=released_objects.begin(); k!=released_objects.end(); ++k)
          ps += *k;
      
        s_str = "CanGrasp_" + i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), s_str);
        
        e.s = v_i - vertices.begin();
          
        t_str = "Grasped_" + i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), t_str);

        if(v_i==vertices.end())
        {
          vertices.push_back(t_str);

          e.t = vertices.size()-1;
        }
        else
          e.t = v_i - vertices.begin();
      }
      else if( !strcmp(i->op.c_str(), "UNGRASP") )
      {
        std::string ps;
        for(std::vector<std::string>::const_iterator k=released_objects.begin(); k!=released_objects.end(); ++k)
          ps += *k;
      
        s_str = "CanRelease_" + i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), s_str);
        
        e.s = v_i - vertices.begin();
        
        t_str = "Released_" + i->args.at(1) + "[" + ps + "]";
        
        v_i = std::find(vertices.begin(), vertices.end(), t_str);

        if(v_i==vertices.end())
        {
          vertices.push_back(t_str);

          e.t = vertices.size()-1;
        }
        else
          e.t = v_i - vertices.begin();
        
        // For the book-keeper
        released_objects.push_back(i->args.at(1));
        released_objects.push_back(".");// Put a delimiter to make parsing easier.
      }
      
      e.w = boost::lexical_cast<double>(i->w);// Put initial weights = 1.0
      e.flag = 0;
      e.name = i->id();
          
      std::vector<Edge>::iterator e_it;
      e_it = std::find(edges.begin(), edges.end(), e);
      
      if(e_it==edges.end())
      {
        edges.push_back(e);
      }
    }// End of: each HighLevelAction in a task plan
  }// End of: for each task plan
  
  // Write the vertices and the edges
  std::string tmg_path = planner_manager_path_ + "/tmg/tmg.dat";
  
  std::ofstream graph_data;
  graph_data.open(tmg_path.c_str());
  
  graph_data << "v" << std::endl;
  for(std::vector<std::string>::const_iterator i=vertices.begin(); i!=vertices.end(); ++i)
  {
    graph_data << *i << std::endl;
  }
  
  graph_data << "e" << std::endl;
  for(std::vector<Edge>::const_iterator i=edges.begin(); i!=edges.end(); ++i)
  {
    graph_data << i->s << " " << i->t << " " << i->w << " " << i->flag << " " << i->name << std::endl;
  }
  
  graph_data.close();
  return true;
}
};// End of: TaskPlanner class

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh;
  
  TaskPlanner tp(nh);
  
  ros::ServiceServer plan_task_srv;
  plan_task_srv = nh.advertiseService("plan_task", &TaskPlanner::plan_task_cb, &tp);
  ROS_INFO("Ready to plan tasks.");
  
  ros::spin();
  
  ros::shutdown();
  return 0;
}
