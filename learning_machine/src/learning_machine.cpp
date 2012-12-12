#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/graph/depth_first_search.hpp>
#include <boost/algorithm/string.hpp>

#include "learning_machine/Train.h"
#include "learning_machine/Test.h"

#include "data_collector.hpp"


class LearningMachine
{
public:
LearningMachine(ros::NodeHandle nh)
: nh_(nh)
{ }

bool
train_srv_handle(learning_machine::Train::Request& req, learning_machine::Train::Response& res)
{
  std::vector<std::string> tmm_paths;// holds planned tmm paths
  tmm_paths.push_back("/home/vektor/hiroken-ros-pkg/planner_manager/data/tmm.dot");
  
  // Collect samples/training data
  get_samples(tmm_paths);
    
  // Train TODO
  
  return true;  
}

bool
test_srv_handle()
{
  // TODO
  return true;
}
private:
void
get_samples(const std::vector<std::string>& tmm_paths)
{
  ROS_DEBUG_STREAM("tmm_paths.size()= " << tmm_paths.size());
  for(std::vector<std::string>::const_iterator i= tmm_paths.begin(); i!=tmm_paths.end(); ++i)
  {
    // Read the planned tmm
    TaskMotionMultigraph tmm;
    boost::dynamic_properties tmm_dp;
    
    tmm_dp.property("vertex_id", get(vertex_name, tmm));
    
    tmm_dp.property("label", get(edge_name, tmm));
    tmm_dp.property("weight", get(edge_weight, tmm));
    tmm_dp.property("jspace", get(edge_jspace, tmm)); 
    tmm_dp.property("color", get(edge_color,tmm));
    tmm_dp.property("srcstate", get(edge_srcstate,tmm));
        
    std::ifstream tmm_dot(i->c_str());
    read_graphviz(tmm_dot, tmm, tmm_dp, "vertex_id");
    
    // Remove more-expensive edges, remove parallelism
    std::set<TMMEdge> tobe_removed_edges;
    
    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
    for(boost::tie(vi,vi_end) = vertices(tmm); vi!=vi_end; ++vi)
    {
      std::map<TMMVertex,TMMEdge> tv_e_map;
      
      graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
      for(tie(oei,oei_end) = out_edges(*vi,tmm); oei!=oei_end; ++oei )
      {
        std::map<TMMVertex,TMMEdge>::iterator it;
        bool inserted;
        
        tie(it,inserted) = tv_e_map.insert( std::make_pair(target(*oei,tmm),*oei) );
        
        if(!inserted)
        {
          if(get(edge_weight,tmm,*oei) < get(edge_weight,tmm,it->second))
          {
            tobe_removed_edges.insert(it->second);

            it->second = *oei;
          }
          else
          {
            tobe_removed_edges.insert(*oei);
          }
        }
      }
    }// end of: For each vertex in tmm
    
    for(std::set<TMMEdge>::const_iterator i=tobe_removed_edges.begin(); i!=tobe_removed_edges.end(); ++i)
      remove_edge(*i,tmm);
    ROS_DEBUG("Parallelism: removed");
    
    // Filter only the planned edge 
    PlannedEdgeFilter<TMMEdgeColorMap> planned_edge_filter( get(edge_color, tmm) );
    typedef filtered_graph< TaskMotionMultigraph, PlannedEdgeFilter<TMMEdgeColorMap> > PlannedTMM;

    PlannedTMM p_tmm(tmm, planned_edge_filter);
    ROS_DEBUG_STREAM("num_vertices(p_tmm)= " << num_vertices(p_tmm));
    ROS_DEBUG_STREAM("num_edges(p_tmm)= " << num_edges(p_tmm));
    
    // Write the filtered tmm: p_tmm
    boost::dynamic_properties p_tmm_dp;
  
    p_tmm_dp.property( "vertex_id",get(vertex_name,p_tmm) );
    p_tmm_dp.property( "label",get(edge_name, p_tmm) );
    p_tmm_dp.property( "weight",get(edge_weight, p_tmm) );
    p_tmm_dp.property( "jspace",get(edge_jspace, p_tmm) );
    p_tmm_dp.property( "color",get(edge_color, p_tmm) );
    p_tmm_dp.property( "srcstate",get(edge_srcstate,p_tmm) );

    std::string p_tmm_dot_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/p_tmm.dot";
    ofstream p_tmm_dot;
    p_tmm_dot.open(p_tmm_dot_path.c_str());
      
    write_graphviz_dp( p_tmm_dot, p_tmm, p_tmm_dp, std::string("vertex_id"));
    p_tmm_dot.close();
    
    // Do dfs to get planned paths. First, make this multigraph to be a graph by removing more expensive edges
    DataCollector dc( &tr_data_,get_feature_names() );
    
    PlannedTMM::vertex_descriptor root;
    boost::graph_traits<PlannedTMM>::vertex_iterator vj, vj_end;
    for(boost::tie(vj,vj_end) = vertices(p_tmm); vj!=vj_end; ++vj)
    {
      if( !strcmp(get(vertex_name,p_tmm,*vj).c_str(),"MessyHome") )
      {
        root = *vj;
        break;
      }
    }
    
    depth_first_visit( p_tmm, root, dc, get(vertex_color, p_tmm));
  }// End of: for each tmm_path
  
  // Write data to a file  
  std::string tr_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/data.csv";
  
  std::ofstream tr_data_out;
  tr_data_out.open( tr_data_path.c_str() );// overwrite
  
  ROS_DEBUG_STREAM("tr_data_.size()= " << tr_data_.size());
  for(Data::const_iterator i=tr_data_.begin(); i!=tr_data_.end(); ++i)
  {
    // Write input=feature values
    for(Input::const_iterator j=i->first.begin(); j!=i->first.end(); ++j)
    {
      tr_data_out << *j << ",";
    }
    
    tr_data_out << i->second << std::endl;
  }
  
  ROS_DEBUG("Samples file: created");
  tr_data_out.close();
}
//! This gets the upper bound of input-feature size
/*!
  More ...
*/
std::vector<std::string>
get_feature_names()
{
  std::vector<std::string> feature_names;
  
  // Read from a csv file containing metadata
  std::string metadata_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/metadata.csv"; // Note that the metadata must only contain 1 line at most.
  std::ifstream metadata_in(metadata_path.c_str());
  
  if ( metadata_in.is_open() )
  {
    std::string metadata;
    
    getline(metadata_in, metadata);// Read only the first line; the only line here
    metadata_in.close();
    
    // Parse the metadata, put into a vector
    boost::split( feature_names, metadata, boost::is_any_of(",") );

    // Note that eventhough there is no "," or the metadata is empty, the resulted vector still has 1 element which is an empty string.
    if( !strcmp(feature_names.at(0).c_str(), std::string("").c_str())  )
    {
        ROS_ERROR("metadata file is corrupt.");
        feature_names.erase(feature_names.begin());
    }
    else
    {
      // Remove the OUT label
      std::vector<std::string>::iterator OUT_it;
      OUT_it = std::find(feature_names.begin(), feature_names.end(), "OUT");
      if(OUT_it!=feature_names.end())
        feature_names.erase(OUT_it);
    }
  }
  else 
  {
    ROS_ERROR("Unable to open metadata file");
  }
  
  return feature_names;
}
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

Data tr_data_;
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "learning_machine");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  LearningMachine learner(nh);
  
  ros::ServiceServer train_srv;
  train_srv = nh.advertiseService("/train", &LearningMachine::train_srv_handle, &learner);

//  ros::ServiceServer test_srv;
//  test_srv = nh.advertiseService("/test", &LearningMachine::test_srv_handle, &learner);
  ROS_INFO("learner: spinning...");
  ros::spin();
  
  return 0;
}
