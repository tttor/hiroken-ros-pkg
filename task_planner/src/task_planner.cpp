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
  task_planner_path_ = ".";
  if( !ros::param::get("/task_planner_path", task_planner_path_) )
    ROS_WARN("Can not get /task_planner_path, use the default value instead");
    
  XmlRpc::XmlRpcValue v;
  nh_.param("rbt_ids", v, v);
  for(int i=0; i < v.size(); i++)
  {
    std::string v_str = v[i];
    
    // Parsing v.at(i),e.g. RARM:rarm,rarm_U_chest
    std::vector<std::string> parts;
    
    boost::split( parts,v_str, boost::is_any_of(":") );
    std::string rbt_id = parts.at(0);
    
    std::vector<std::string> jspaces;    
    boost::split( jspaces,parts.at(1), boost::is_any_of(",") );
    
    // Assign
    std::set<std::string> jspace_set;
    for(std::vector<std::string>::const_iterator j=jspaces.begin(); j!=jspaces.end(); ++j)
      jspace_set.insert(*j);
    
    rbt_jspaces_map_.insert( std::make_pair(rbt_id,jspace_set) );
  }
  
//  // For testing only, should be commented on normal runs
//  plan(2);
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
  return plan( req.objects.size() );
}
//! Brief ...
/*!
  This is the core function in the TaskPlanner class.
  TODO Calling to HTN planner should be done here.
  
  For now, up to Oct 13, 2012, the communication with a CommonLisp-based HTN plannner is done via a file; hardcoded!
  In the same vein, the commomunication with the planner manager is also done via a file; hardcoded!.
  
  Note that the plan structure that is writtent in a plan file is different between single-arm and dual-arm, why?
  in that for dual-arm plans we donot have to: Remove the head "(" and the tail ")"; the most outer ones
  
  Note that intentionally, we make changes in the operator args list:
  FROM (!GRASP R CAN1 MESSY-SPOT) TO (!GRASP CAN1 MESSY-SPOT RARM)
  
  \n_obj number of objects, which determines the planning horizon
  \return whether successful
*/
bool
plan(const size_t& n_obj)
{
//  std::string plan_path = task_planner_path_ + "/with_shop2/plans/" ;
  std::string plan_path = task_planner_path_ + "/with_shop2/plans.dual-arm/" ;
  for(size_t i=0; i<n_obj; ++i)
    plan_path += boost::lexical_cast<std::string>(i+1) + ".";
  plan_path += "plan";// the file extension foo.plan
  
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
    ROS_ERROR_STREAM("Unable to open a task plan file from " << plan_path);
    return false;
  }

//  // Remove the head "(" and the tail ")"; the most outer ones
//  boost::trim(lines);
//  boost::erase_head(lines, 1);
//  boost::erase_tail(lines, 1);
  
  // Replacement for an easy splitting; "((" that separates plans
  boost::replace_all(lines, "((", "#");
    
  // Parse into plans
  std::vector<std::string> plans;
  boost::split( plans, lines, boost::is_any_of("#") );
  
  // Erase the first element of the vector plans, it contains an empty string, because there is a # at the beginning
  plans.erase( plans.begin() );
  
  ROS_DEBUG_STREAM("plans.size()=" << plans.size());
//  for(std::vector<std::string>::iterator i=plans.begin(); i!=plans.end(); ++i)
//    ROS_DEBUG_STREAM(*i);
  
  // Construct a task graph
  TaskGraph g;// for the task graph.
  std::map<std::string, TGVertex> name_vertex_map;
  
  for(std::vector<std::string>::iterator plan_it=plans.begin(); plan_it!=plans.end(); ++plan_it)
  {
    // Replace the first "!" with "(!" in each plan AND erase the tail ")"
    boost::trim(*plan_it);
    boost::replace_first(*plan_it, "!", "(!");
    boost::erase_tail(*plan_it,1);
    
    // Parsing for a plan
    std::vector<std::string> raw_actions;
    std::vector<Action> actions;
      
    boost::split( raw_actions, *plan_it, boost::is_any_of("(!"), boost::algorithm::token_compress_on );
    raw_actions.erase( raw_actions.begin() );
    
//    ROS_DEBUG_STREAM("raw_actions.size()=" << raw_actions.size());
//    for(std::vector<std::string>::iterator i=raw_actions.begin(); i!=raw_actions.end(); ++i)
//      ROS_DEBUG_STREAM(*i);
    
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
    
    // Remove unnecesary TRANSITs here, if any
    // e.g. TRANSIT_RARM_TIDY-SPOT_HOME -> TRANSIT_RARM_HOME_MESSY-SPO
    std::vector< std::vector<Action>::iterator > tobe_erased_actions;
    for(std::vector<Action>::iterator i=actions.begin(); i!=actions.end()-1; ++i)
    {
      std::string op = i->op;
      std::string next_op = (i+1)->op;
      
      if( !strcmp(op.c_str(),"TRANSIT") and !strcmp(next_op.c_str(),"TRANSIT") )
      {
        std::string rbt_id = i->args.at(0);
        std::string next_rbt_id = (i+1)->args.at(0);
        
        if( !strcmp(rbt_id.c_str(),next_rbt_id.c_str()) )
        {
          Action act;
          
          act.op = std::string("TRANSIT");
          
          act.args.resize(3);
          act.args.at(0) = i->args.at(0);// rbt_id
          act.args.at(1) = i->args.at(1);// from_location
          act.args.at(2) = (i+1)->args.at(2);// to_location
          
          // Replace
          *i = act;
          
          // Prepare for deletions
          tobe_erased_actions.push_back( (i+1) );
        }
      }
    }
    
    for(std::vector< std::vector<Action>::iterator >::const_iterator i=tobe_erased_actions.begin(); i!=tobe_erased_actions.end(); ++i)
    {
      size_t n_deletion_so_far = i-tobe_erased_actions.begin();
      
      actions.erase( (*i)-(n_deletion_so_far) ); 
    }
    
    ROS_DEBUG_STREAM("actions.size()=" << actions.size());
    for(std::vector<Action>::iterator i=actions.begin(); i!=actions.end(); ++i)
      ROS_DEBUG_STREAM(i->id());
    
    // For book-keeping which object has been released (put at the tidy-spot)
    // This is necessary to differentiate nodes.
    // note that the contents of the released_objects vector include some dots, which is there to make parsing easier
    std::vector<std::string> released_objects;
    
    for(std::vector<Action>::iterator i=actions.begin(); i!=actions.end(); ++i)
    {
      // Infer the states before and after an action; Infer the connectivity in plans from SHOP2 planner
      std::vector<Action>::iterator pre_i = i-1;
      std::vector<Action>::iterator post_i = i+1;
      
      std::string source_vertex_name;
      std::string target_vertex_name;
      
      tie(source_vertex_name, target_vertex_name) = infer(i, pre_i, post_i, n_obj, &released_objects);
      
      // Build the task graph
      TGEdge edge;
      std::map<std::string, TGVertex>::iterator name_vertex_map_it;
      bool inserted;
      
      TGVertex source_vertex;
      TGVertex target_vertex;
      
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
      /*
      The weight from the task plan is ignored because
      1) it is uniform for all edges so it is meaningless
      2) I think it is meant to be the geometric planning cost for implementing an action, where here we definitely focus on calculating that cost.
      */
    }// End of: each Action in a task plan
  }// End of: for each task plan

  // Write the task_graph.dot
  std::string data_path = ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  std::string task_graph_dot_path = data_path + "/task_graph.dot";
  
  std::ofstream task_graph_dot;
  task_graph_dot.open(task_graph_dot_path.c_str());
  
  write_graphviz( task_graph_dot, g
                , VertexPropWriter_1<TGVertexNameMap>( get(vertex_name, g) )
                , EdgePropWriter_1<TGEdgeNameMap, TGEdgeWeightMap>( get(edge_name, g), get(edge_weight, g) )
                );  
  
  task_graph_dot.close();

  // Convert the task graph to the Task Motion Multigraph
  TaskMotionMultigraph tmm;
  tmm = convert_tg2tmm(g);
  
  // Write the tmm.dot
  std::string tmm_dot_path = data_path + "/vanilla_tmm.dot";
  
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
convert_tg2tmm(const TaskGraph& tg)
{
  // Build the TMM=====================================================================================
  TaskMotionMultigraph tmm;
  std::map<std::string,TMMVertex> name_vertex_map;
  
  graph_traits<TaskGraph>::edge_iterator ei, ei_end;
  for(tie(ei,ei_end) = edges(tg); ei != ei_end; ++ei)
  {
    // The TMM only consider an edge that requires motion planning    
    std::vector<std::string> edge_name_parts;
    boost::split( edge_name_parts, get(edge_name, tg, *ei), boost::is_any_of("_"), boost::token_compress_on );
    
    if( !strcmp(edge_name_parts.at(0).c_str(), std::string("GRASP").c_str()) or !strcmp(edge_name_parts.at(0).c_str(), std::string("UNGRASP").c_str()) )
      continue;
    
    // For keeping only unique vertex names
    std::map<std::string, TMMVertex>::iterator name_vertex_map_it;
    bool inserted;
    
    // Determine the source vertex=========================================================================
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
    
    // Determine the target vertex=======================================================================
    TMMVertex target_vertex;
    std::vector<std::string> target_vertex_name_vec;// a vector because if a bypassing happens, there may be more than one adjacent vertices
    
    target_vertex_name_vec.push_back( get(vertex_name,tg,target(*ei,tg)) );
    
    // The target_vertex_name may be either CanGrasp_O_R[] or Grasped_O_R[] or CanRelease_O_R[] or Released_O_R[] or TidyHome or TmpHome_RBTID[xxx]
    std::vector<std::string> target_vertex_name_parts;
    boost::split( target_vertex_name_parts,target_vertex_name_vec.at(0),boost::is_any_of("_"),boost::token_compress_on );
    
    std::string main_state_str = target_vertex_name_parts.at(0);
    if( !strcmp(main_state_str.c_str(),"CanGrasp") or !strcmp(main_state_str.c_str(),"Grasped") or !strcmp(main_state_str.c_str(),"CanRelease") or !strcmp(main_state_str.c_str(),"Released") )
    {
      target_vertex_name_vec.clear();
      
      // The vertices that are adjacent with the target vertex of this edge are used because the target vertex of this edge is bypassed
      graph_traits<TaskGraph>::adjacency_iterator avi, avi_end;
      for(tie(avi, avi_end) = adjacent_vertices( target(*ei,tg),tg ); avi != avi_end; ++avi)
        target_vertex_name_vec.push_back( get(vertex_name, tg, *avi) );
    }
    else// must be either TidyHome or TmpHome_RBTID[xxx]
    { }
    
    for(std::vector<std::string>::const_iterator i=target_vertex_name_vec.begin(); i!=target_vertex_name_vec.end(); ++i)
    {
      std::string target_vertex_name = *i;
      
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
      
      // Build the edges======================================================================================================
      // Get the eof/rbt_id from either source_vertex_name or targer_vertex_name
      // Examples of source/targer_vertex_name: MessyHome, TidyHome, GRASPED_CAN1_RARM[], RELEASED_CAN1_RARM[]
      // NOTE that the rbt_id is limited to only 4 characters at most, the rest is truncated.
      std::string raw_rbt_id;
      
      std::vector<std::string> vertex_name_parts;
      boost::split( vertex_name_parts,get(vertex_name,tmm,source_vertex),boost::is_any_of("_"),boost::token_compress_on );

      if(vertex_name_parts.size()==3)//NOTE that a valid parsing iff ...parts.size()==3
      {
        raw_rbt_id = vertex_name_parts.at(2);// with "[xxx]"
      }
      else
      {
        boost::split( vertex_name_parts,get(vertex_name,tmm,target_vertex),boost::is_any_of("_"),boost::token_compress_on );
        raw_rbt_id = vertex_name_parts.at(2);// with "[xxx]"
      }
      boost::trim(raw_rbt_id);
      
      // The rbt_id is limited to only 4 characters at most and at least, the rest, if any, is truncated 
      std::string rbt_id = raw_rbt_id.substr(0,4);
      
      for(std::set<std::string>::const_iterator j=rbt_jspaces_map_[rbt_id].begin(); j!=rbt_jspaces_map_[rbt_id].end(); ++j)
      {
        if(j->length()==0)
        {
          ROS_WARN("jspace is blank, the rbt_id may be undefined");
          continue;
        }
        
        // Force to return TidyHome with the full joint space
        if( !strcmp(get(vertex_name,tmm,target_vertex).c_str(),"TidyHome") )
          if( !strcmp(j->c_str(),"rarm") or !strcmp(j->c_str(),"larm") )
            continue;
                    
        TMMEdge edge;
        tie(edge, inserted) = add_edge(source_vertex, target_vertex, tmm);
        
        put( edge_name, tmm, edge, get(edge_name, tg, *ei) );
        put( edge_weight,tmm,edge,get(edge_weight,tg,*ei) + get(edge_weight,tg,*ei) );
        
        put( edge_jspace, tmm, edge, *j);
      }
    }
  }// end of: FOR each edge in a task graph tg
  
  return tmm;
}
//! Infer the connectivity in plans from SHOP2 planner
/*!
  More...
  
  \param &i the iterator of an action
  \param &pre_i the pre-iterator of an action
  \param &post_i the post-iterator of an action
  \param &n_obj the number of involved objects, which is equal to the maximum number of released objects
  \param *released_objects a bookeeper variable storing which object has been released
  \return a pair of source_vertex_name and targer_vertex_name
*/
std::pair<std::string, std::string>
infer(const std::vector<Action>::iterator& i,const std::vector<Action>::iterator& pre_i,const std::vector<Action>::iterator& post_i,const size_t& n_obj,std::vector<std::string>* released_objects)
{
  std::string source_vertex_name;
  std::string target_vertex_name;
  
  // ==================================================================================="TRANSIT"
  if( !strcmp(i->op.c_str(), "TRANSIT") )
  {
    // Infer the source vertex name
    std::string source_loc = i->args.at(1);
    
    if( !strcmp(source_loc.c_str(), "HOME") )
    {
      if(released_objects->empty())// have not yet tidied anything
      {
        source_vertex_name = "MessyHome";
      }
      else
      {
        std::string ps;
        for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
          ps += *k;
        
        std::string rbt_id = i->args.at(0);
        for(std::map< std::string,std::set<std::string> >::const_iterator i=rbt_jspaces_map_.begin(); i!=rbt_jspaces_map_.end(); ++i)
        {
          // update rbt_id with another value, only valid iff there are only 2 rbt_ids
          if( strcmp(i->first.c_str(),rbt_id.c_str()) )//if *i is not the same as rbt_id, then ...
          {
            rbt_id = i->first;
            break;
          }
        }
            
        source_vertex_name = "TmpHome_" + rbt_id + "[" + ps + "]";
      }
    }
    else
    {
      // Assume that an Act "TRANSIT" is always after a state "Released_XXX"
      
      //  Note that k is incremented up to just before end()-2 because .....
      // (1) the goal here is to get the source state, which is before "Released_XXX" happens, however, we are here after it, therefore the just released objects should be ignored
      // (2) note that the contents of the released_objects vector include some dots, which is there to make parsing easier
      std::string ps;
      for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=(released_objects->end()-2); ++k)
        ps += *k;
      
      std::string released_obj_id = pre_i->args.at(0);
      std::string released_rbt_id = pre_i->args.at(2);
      
      source_vertex_name = "Released_" + released_obj_id + "_" + released_rbt_id + "[" + ps + "]";
    }
    
    // Infer the target_vertex name
    std::string target_loc = i->args.at(2);
    
    if( !strcmp(target_loc.c_str(), "HOME") )
    {
      if((released_objects->size()/2) == n_obj)// Divide by two because the contents of the released_objects vector include some dots, see more on its note
      {
        target_vertex_name = "TidyHome";
      }
      else
      {
        std::string ps;
        for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
          ps += *k;
        
        std::string rbt_id = i->args.at(0);
        
        target_vertex_name = "TmpHome_" + rbt_id + "[" + ps + "]";
      }
    }
    else
    {
      // Assume that an Act "TRANSIT" is always followed by a state "CanGrasp_XXX"
      
      std::string ps;
      for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
        ps += *k;
      
      std::string cangrasp_obj_id = post_i->args.at(0);
      std::string cangrasp_rbt_id = post_i->args.at(2);
      
      target_vertex_name = "CanGrasp_" + cangrasp_obj_id + "_" + cangrasp_rbt_id + "[" + ps + "]";
    }
  }
  
  //==================================================================================="TRANSFER"
  if( !strcmp(i->op.c_str(), "TRANSFER") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
  
    // Assume that an Act "TRANSFER" is always following the state of "Grasped_XXX".
    std::string grasped_obj_id = i->args.at(1);
    std::string grasped_rbt_id = i->args.at(0);
    
    source_vertex_name = "Grasped_" + grasped_obj_id + "_" + grasped_rbt_id + "[" + ps + "]";
            
    // Assume that "TRANSFER" is always followed by a state "CanRelease_XXX".
    std::string canrelease_obj_id = post_i->args.at(0);
    std::string canrelease_rbt_id = post_i->args.at(2);
    
    target_vertex_name = "CanRelease_" + canrelease_obj_id + "_" +  canrelease_rbt_id + "[" + ps + "]";
  }
  
  //==================================================================================="GRASP"
  if( !strcmp(i->op.c_str(), "GRASP") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
  
    std::string cangrasp_obj_id = i->args.at(0);
    std::string cangrasp_rbt_id = i->args.at(2);
    
    source_vertex_name = "CanGrasp_" + cangrasp_obj_id + "_" + cangrasp_rbt_id + "[" + ps + "]";
    
    std::string grasped_obj_id = i->args.at(0);
    std::string grasped_rbt_id = i->args.at(2);    
    
    target_vertex_name = "Grasped_" + grasped_obj_id + "_" + grasped_rbt_id + "[" + ps + "]";
  }
  
  //==================================================================================="UNGRASP"
  if( !strcmp(i->op.c_str(), "UNGRASP") )
  {
    std::string ps;
    for(std::vector<std::string>::const_iterator k=released_objects->begin(); k!=released_objects->end(); ++k)
      ps += *k;
    
    std::string canrelease_obj_id = i->args.at(0);
    std::string canrelease_rbt_id = i->args.at(2);
    
    source_vertex_name = "CanRelease_" + canrelease_obj_id + "_" + canrelease_rbt_id + "[" + ps + "]";
  
    std::string released_obj_id = i->args.at(0);
    std::string released_rbt_id = i->args.at(2);
    
    target_vertex_name = "Released_" + released_obj_id + "_" + released_rbt_id + "[" + ps + "]";
    
    // For the book-keeper
    released_objects->push_back(released_obj_id);
    released_objects->push_back(".");// Put a delimiter to make parsing easier.
  }
  
  return std::make_pair(source_vertex_name, target_vertex_name);
}

//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

std::string task_planner_path_;

std::map< std::string,std::set<std::string> > rbt_jspaces_map_;

};// End of: TaskPlanner class

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh;
  
//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  TaskPlanner tp(nh);
  
  ros::ServiceServer plan_task_srv;
  plan_task_srv = nh.advertiseService("plan_task", &TaskPlanner::handle_plan_task_srv, &tp);
 
  ROS_INFO("TaskPlanner: Up and Running..."); 
  ros::spin();
  
  return 0;
}
