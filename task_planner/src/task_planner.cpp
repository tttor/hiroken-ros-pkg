#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>

#include "task_planner/PlanTask.h"

#include "tmm_utils.hpp"

using namespace boost;
using namespace std;

struct Action
{
  std::string op;
  std::vector<std::string> args;
  double w;
  
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
  data_path_ = ".";
  if( !ros::param::get("/data_path", data_path_) )
    ROS_WARN("Can not get /data_path, use the default value instead");
  
  ROS_INFO("TaskPlanner: Up and Running...");
}

~TaskPlanner()
{ }

//! Brief ...
/*!
  This handles the service named plan_task.
  
  \param req A request: arm_navigation_msgs/CollisionObject[] objects
  \param res A response, which is nothing here
  \return whether successful
*/
bool
handle_plan_task_srv(task_planner::PlanTask::Request& req, task_planner::PlanTask::Response& res)
{
  return plan(req.objects);
}
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
  std::string plan_path = "/home/vektor/hiroken-ros-pkg/task_planner/with_shop2/plans/";
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
  
  // Construct a task graph
  TaskGraph g;// for the task graph.
  std::map<std::string, TGVertex> name_vertex_map;
  
  for(std::vector<std::string>::iterator plan_it=plans.begin(); plan_it!=plans.end(); ++plan_it)
  {
    // Replace "!" with "(!" in each plan AND erase the tail ")"
    boost::trim(*plan_it);
      
    boost::replace_first(*plan_it, "!", "(!");
    boost::erase_tail(*plan_it,1);
    
    // Parsing for plans.tmg
    std::vector<std::string> raw_actions;
    std::vector<Action> actions;
      
    boost::split( raw_actions, *plan_it, boost::is_any_of("(!"), boost::algorithm::token_compress_on );
    raw_actions.erase( raw_actions.begin() );
    
    for(std::vector<std::string>::iterator i=raw_actions.begin(); i!=raw_actions.end(); ++i)
    {
      boost::erase_all( *i, ")" );
      boost::trim(*i);
      
      std::vector<std::string> hla_parts;
      boost::split( hla_parts, *i, boost::is_any_of(" "), boost::token_compress_on );
      
      Action hla;
      hla.op = hla_parts.at(0);
      for(std::vector<std::string>::iterator j=hla_parts.begin()+1; j!=hla_parts.end(); ++j)
      {
        hla.args.push_back(*j);
      }
      hla.args.erase( hla.args.end()-1 );
      
      hla.w = boost::lexical_cast<double>( hla_parts.at(hla_parts.size()-1) );
      
      actions.push_back(hla);
    }
    
    // For book-keeping which object has been released.
    // This is necessary to differentiate nodes.
    std::vector<std::string> released_objects;
    
    for(std::vector<Action>::iterator i=actions.begin(); i!=actions.end(); ++i)
    {
      TGVertex source_vertex;
      TGVertex target_vertex;
      
      std::string source_vertex_name;
      std::string target_vertex_name;
      
      TGEdge edge;
      std::map<std::string, TGVertex>::iterator name_vertex_map_it;
      bool inserted;
      
      std::vector<Action>::iterator pre_i = i-1;
      std::vector<Action>::iterator post_i = i+1;
      
      tie(source_vertex_name, target_vertex_name) = infer(i, pre_i, post_i, &released_objects);
      
      tie(name_vertex_map_it, inserted) = name_vertex_map.insert( std::make_pair(source_vertex_name, TGVertex()) );
      if(inserted)
      {
        source_vertex = add_vertex(g);
        put(vertex_name, g, source_vertex, source_vertex_name);
        
        name_vertex_map_it->second = source_vertex;
      }
      else
      {
        source_vertex = name_vertex_map_it->second;
      }
      
      tie(name_vertex_map_it, inserted) = name_vertex_map.insert( std::make_pair(target_vertex_name, TGVertex()) );
      if(inserted)
      {
        target_vertex = add_vertex(g);
        put(vertex_name, g, target_vertex, target_vertex_name);
        
        name_vertex_map_it->second = target_vertex;
      }
      else
      {
        target_vertex = name_vertex_map_it->second;
      }
      
      tie(edge, inserted) = add_edge(source_vertex, target_vertex, g);
      put(edge_name, g, edge, i->id());
      put(edge_weight, g, edge, 0.);//put(edge_weight, g, edge, i->w);
    }// End of: each Action in a task plan
  }// End of: for each task plan

  // Write the task_graph.dot
  std::string task_graph_dot_path = data_path_ + "/task_graph.dot";
  
  std::ofstream task_graph_dot;
  task_graph_dot.open(task_graph_dot_path.c_str());
  
  write_graphviz( task_graph_dot, g
                , VertexPropWriter_1<TGVertexNameMap>( get(vertex_name, g) )
                , EdgePropWriter_1<TGEdgeNameMap, TGEdgeWeightMap>( get(edge_name, g), get(edge_weight, g) )
                );  
  
  task_graph_dot.close();

  // Convert to Task Motion Multigraph
  TaskMotionMultigraph tmm;
  tmm = convert_tg_tmm(g);
  
//  create_metadata(tmm,objs.size());
  
  return true;
}

private:
//! A helper function
/*!
  Create metadata file that contains input-feature names.
  Should be rarely used.
  
  \param tmm
  \param &n number of movable objects
*/
void
create_metadata(const TaskMotionMultigraph& tmm, const size_t& n)
{
  ROS_WARN("Creating metadata...");
  std::set<std::string> sym_feature_names;
  
  boost::graph_traits<TaskMotionMultigraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(tmm); ei != ei_end; ++ei)
  {
    sym_feature_names.insert( get(edge_name,tmm,*ei) );
  }
  
  std::vector<std::string> geo_feature_names;
  std::string obj_id = "CAN";
  
  for(size_t i=1; i<=n; ++i)
  {
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".x");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".y");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".z");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qx");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qy");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qz");
    geo_feature_names.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qw");
  }
  
  // Write the metadata
  std::string metadata_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/metadata.csv";
  
  std::ofstream metadata_out;
  metadata_out.open(metadata_path.c_str());// overwrite
  
  for(std::set<std::string>::const_iterator i=sym_feature_names.begin(); i!=sym_feature_names.end(); ++i)
  {
    metadata_out << *i << ",";
  }
  for(std::vector<std::string>::const_iterator i=geo_feature_names.begin(); i!=geo_feature_names.end(); ++i)
  {
    metadata_out << *i << ",";
  }
  metadata_out << "OUT";
  
  metadata_out.close();
  
  ROS_WARN("metadata: created.");
}
//! Convert a  task_graph into a task motion multigraph
/*!
  More...
  
  \param &tg a task graph
  \return the resulted tmm
*/
TaskMotionMultigraph
convert_tg_tmm(const TaskGraph& tg)
{
    // Set joint_spaces 
  std::list<std::string> jspaces;
  jspaces.push_back("rarm");
  jspaces.push_back("rarm_U_chest");
  
  TaskMotionMultigraph tmm;
  std::map<std::string, TMMVertex> name_vertex_map;
  
  graph_traits<TaskGraph>::edge_iterator ei, ei_end;
  for(tie(ei,ei_end) = edges(tg); ei != ei_end; ++ei)
  {
    std::string name;
    name = get(edge_name, tg, *ei);
    
    std::vector<std::string> name_parts;
    boost::split( name_parts, name, boost::is_any_of("_"), boost::token_compress_on );
    
    if( !strcmp(name_parts.at(0).c_str(), std::string("GRASP").c_str()) or !strcmp(name_parts.at(0).c_str(), std::string("UNGRASP").c_str()) )
      continue;

    std::map<std::string, TMMVertex>::iterator name_vertex_map_it;
    bool inserted;
    
    // Determine the source vertex
    TMMVertex source_vertex;
    std::string source_vertex_name = get(vertex_name, tg, source(*ei,tg));
    
    tie(name_vertex_map_it, inserted) = name_vertex_map.insert(  std::make_pair( source_vertex_name,TMMVertex() )  );
    if(inserted)
    {
      source_vertex = add_vertex(tmm);
      put(vertex_name, tmm, source_vertex, source_vertex_name);
      
      name_vertex_map_it->second = source_vertex;
    }
    else
    {
      source_vertex = name_vertex_map_it->second;
    }
    
    // Determine the target vertex
    TMMVertex target_vertex;
    std::string target_vertex_name;     
    
    bool goal_vertex = false;
    if( out_degree(target(*ei,tg), tg)==0 )
      goal_vertex = true;
    
    graph_traits<TaskGraph>::adjacency_iterator avi, avi_end;
    for(tie(avi, avi_end) = adjacent_vertices( target(*ei,tg),tg ); (avi != avi_end) or goal_vertex; ++avi)
    {
      if(goal_vertex)
        target_vertex_name = get(vertex_name, tg, target(*ei,tg));
      else
        target_vertex_name = get(vertex_name, tg, *avi);
      
      tie(name_vertex_map_it, inserted) = name_vertex_map.insert(  std::make_pair( target_vertex_name,TMMVertex() )  );
      if(inserted)
      {
        target_vertex = add_vertex(tmm);
        put(vertex_name, tmm, target_vertex, target_vertex_name);
              
        name_vertex_map_it->second = target_vertex;
      }
      else
      {
        target_vertex = name_vertex_map_it->second;
      }
      
      for(std::list<std::string>::const_iterator i=jspaces.begin(); i!=jspaces.end(); ++i)
      {
        TMMEdge edge;
        tie(edge, inserted) = add_edge(source_vertex, target_vertex, tmm);
        
        put( edge_name, tmm, edge, get(edge_name, tg, *ei) );
        put(  edge_weight, tmm, edge, get(edge_weight, tg, *ei) + (!goal_vertex * get(edge_weight,tg,*ei))  );// Plus 1*edge_weight because it bypasses an edge
        
        put( edge_jspace, tmm, edge, *i);
      }
      
      if(goal_vertex) break;// Do it exactly once.
    }
  }// end of: FOR each edge in a task graph tg
  
  // Write the tmm.dot
  std::string tmm_dot_path = data_path_ + "/vanilla_tmm.dot";
  
  std::ofstream tmm_dot;
  tmm_dot.open(tmm_dot_path.c_str());
      
//  write_graphviz( tmm_dot, tmm
//                , VertexPropWriter_1<TMMVertexNameMap>( get(vertex_name, tmm) )
//                , TMMEdgePropWriter<TMMEdgeNameMap, TMMEdgeWeightMap, TMMEdgeJspaceMap>( get(edge_name, tmm), get(edge_weight, tmm), get(edge_jspace, tmm) )
//                );  

  boost::dynamic_properties dp;
  
  dp.property("vertex_id", get(vertex_name, tmm));
  
  dp.property("label", get(edge_name, tmm));
  dp.property("weight", get(edge_weight, tmm));
  dp.property("jspace", get(edge_jspace, tmm));
  
  write_graphviz_dp( tmm_dot, tmm, dp, std::string("vertex_id"));
  
  tmm_dot.close();

  return tmm;
}
//! Infer the connectivity in plans from SHOP2 planner
/*!
  More...
  
  \param &i the iterator of an action
  \param &pre_i the pre-iterator of an action
  \param &post_i the post-iterator of an action
  \param *released_objects a bookeeper variable storing which object has been released
  \return a pair of source_vertex_name and targer_vertex_name
*/
std::pair<std::string, std::string>
infer(const std::vector<Action>::iterator& i, const std::vector<Action>::iterator& pre_i, const std::vector<Action>::iterator& post_i, std::vector<std::string>* released_objects)
{
  std::string source_vertex_name;
  std::string target_vertex_name;
  
  if( !strcmp(i->op.c_str(), "TRANSIT") )
  {
    // Infer the source vertex name
    if( !strcmp(i->args.at(1).c_str(), "HOME") )
    {
      source_vertex_name = "MessyHome";
    }
    else// Assume that an Act "TRANSIT" is always after a state "Released_XXX"
    {
      std::string ps;//  Note that k is incremented up to 
      for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=(released_objects->end()-2); ++k)
        ps += *k;
        
      source_vertex_name = "Released_" + pre_i->args.at(1) + "[" + ps + "]";
    }
    
    // Infer the target_vertex name
    if( !strcmp(i->args.at(2).c_str(), "HOME") )
    {
      target_vertex_name = "TidyHome";
    }
    else// Assume that an Act "TRANSIT" is always followed by a state "CanGrasp_XXX"
    {
      std::string ps;
      for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
        ps += *k;
      
      target_vertex_name = "CanGrasp_" + post_i->args.at(1) + "[" + ps + "]";
    }
  }
  
  if( !strcmp(i->op.c_str(), "TRANSFER") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
  
    // Assume that an Act "TRANSFER" is always following a state "Grasped_XXX".
    source_vertex_name = "Grasped_" + i->args.at(1) + "[" + ps + "]";
            
    // Assume that "TRANSFER" is always followed by a state "CanRelease_XXX".
    target_vertex_name = "CanRelease_" + post_i->args.at(1) + "[" + ps + "]";
  }
  
  if( !strcmp(i->op.c_str(), "GRASP") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
  
    source_vertex_name = "CanGrasp_" + i->args.at(1) + "[" + ps + "]";
    
    target_vertex_name = "Grasped_" + i->args.at(1) + "[" + ps + "]";
  }
  
  if( !strcmp(i->op.c_str(), "UNGRASP") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
  
    source_vertex_name = "CanRelease_" + i->args.at(1) + "[" + ps + "]";
  
    target_vertex_name = "Released_" + i->args.at(1) + "[" + ps + "]";
    
    // For the book-keeper
    released_objects->push_back(i->args.at(1));
    released_objects->push_back(".");// Put a delimiter to make parsing easier.
  }
  
  return std::make_pair(source_vertex_name, target_vertex_name);
}

//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

std::string data_path_;
};// End of: TaskPlanner class

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh;
  
  TaskPlanner tp(nh);
  
  ros::ServiceServer plan_task_srv;
  plan_task_srv = nh.advertiseService("plan_task", &TaskPlanner::handle_plan_task_srv, &tp);
  
  ros::spin();
  
  return 0;
}
