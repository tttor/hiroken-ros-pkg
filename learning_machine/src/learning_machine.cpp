#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/graph/depth_first_search.hpp>
#include <boost/algorithm/string.hpp>

#include "learning_machine/Train.h"
#include "learning_machine/Test.h"
#include "learning_machine/CreateMetadata.h"

#include "hiro_common/GetManipulability.h"

#include "data.hpp"
#include "tmm_utils.hpp"
#include "data_collector.hpp"

class LearningMachine
{
public:
LearningMachine(ros::NodeHandle nh)
: nh_(nh)
{
  data_path_= ".";
  if( !ros::param::get("/data_path", data_path_) )
    ROS_WARN("Can not get /data_path, use the default value instead");
}

//! This trains _offline_
bool
train_srv_handle(learning_machine::Train::Request& req, learning_machine::Train::Response& res)
{
  // Collect samples/training data
  if( !get_samples(req.tmm_paths) )
  {
    ROS_ERROR("get_samples() failed");
    return false;
  }
  
  // Train TODO
  // Up to now, done in matlab
  
  return true;  
}

bool
create_metadata_srv_handle(learning_machine::CreateMetadata::Request& req,learning_machine::CreateMetadata::Response& res)
{
  if(create_metadata(req.n_obj))
  {
    res.msg = "metadata: created.";
    return true;
  }
  else
  {
    return false;
  }
}

private:
//! A helper function
/*!
  Create metadata file that contains input-feature names a.k.a labels
  Should be rarely used.
  
  WARN: sync. with get_fval() on data_collector.hpp
  
  Labels are defined by the upper bound of planning horizon, here: 5obj.
  
  Labels are in this order:
  x^{g1} : robot joint states;
  x^{g2} : manipulability measure m of robot state  
  x^{g3}: poses of movable objects
  x^{g5}: Cartesian distances (of center of mass) of movable objects' current and final positions
  
  x^{s1}: position of actions
  x^{s2}: length of the path
  x^{s3}: transit transfer centric and 
  x^{s4}: right left arm centric
  
  \param &n_obj number of movable objects
*/
bool
create_metadata(const size_t& n_obj)
{
  ROS_DEBUG("Creating metadata...");
  
  // Init 
  std::string tmm_dot_path;
  tmm_dot_path = std::string("/home/vektor/rss-2013/data/ref/vanilla_tmm."+boost::lexical_cast<std::string>(n_obj)+"M.dot");
  
  std::string metadata_path;
  metadata_path = std::string("/home/vektor/rss-2013/data/ref/metadata."+boost::lexical_cast<std::string>(n_obj)+".csv");

  std::vector<std::string> labels;
  
  // Read the referenced tmm  
  TaskMotionMultigraph tmm;
  boost::dynamic_properties tmm_dp;
  
  tmm_dp.property("vertex_id", get(vertex_name, tmm));
  tmm_dp.property("label", get(edge_name, tmm));
  tmm_dp.property("weight", get(edge_weight, tmm));
  tmm_dp.property("jspace", get(edge_jspace, tmm)); 
  
  std::ifstream tmm_dot(tmm_dot_path.c_str());
  
  if ( !read_graphviz(tmm_dot,tmm,tmm_dp, "vertex_id"))
  {
    ROS_ERROR("read_graphviz() failed.");
    return false;
  }

  // x^{g1} : robot joint states;
  labels.push_back("joint_chest_yaw");
    
  labels.push_back("joint_rshoulder_yaw");
  labels.push_back("joint_rshoulder_pitch");
  labels.push_back("joint_relbow_pitch");
  labels.push_back("joint_rwrist_yaw");
  labels.push_back("joint_rwrist_pitch");
  labels.push_back("joint_rwrist_roll");
    
  labels.push_back("joint_lshoulder_yaw");
  labels.push_back("joint_lshoulder_pitch");
  labels.push_back("joint_lelbow_pitch");
  labels.push_back("joint_lwrist_yaw");
  labels.push_back("joint_lwrist_pitch");
  labels.push_back("joint_lwrist_roll");
  
  // x^{g2} : manipulability measure m of robot state  
  labels.push_back("RARM.m");
  labels.push_back("LARM.m");
  
  // x^{g3}: poses of movable objects
  std::vector<std::string> geo_feature_names;
  std::string obj_id = "CAN";
  
  for(size_t i=1; i<=n_obj; ++i)
  {
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".x");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".y");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".z");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qx");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qy");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qz");
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".qw");
  }
  
  // x^{g5}: Cartesian distances (of center of mass) of movable objects' current and final positions
  for(size_t i=1; i<=n_obj; ++i)
  {
    labels.push_back(obj_id + boost::lexical_cast<std::string>(i) + ".dist");
  }
  
  // x^{s1}: position of actions
  std::set<std::string> sym_label_set;
  
  boost::graph_traits<TaskMotionMultigraph>::edge_iterator ei, ei_end;
  for(boost::tie(ei, ei_end) = edges(tmm); ei != ei_end; ++ei)
  {
    std::string label = get(edge_name,tmm,*ei);
    
    if( sym_label_set.insert(label).second )
      labels.push_back(label);
  }
  
  // x^{s2}: length of the path
  labels.push_back("len");
  
  // x^{s3}: transit transfer centric 
  labels.push_back("TRANSIT-centric");
  labels.push_back("TRANSFER-centric");  
  
  // x^{s4}: right left arm centric
  labels.push_back("LARM-centric");
  labels.push_back("RARM-centric");
  
  // Write the metadata 
  cout << "labels.size()= " << labels.size() << endl;
  
  std::ofstream metadata_out;
  metadata_out.open(metadata_path.c_str());// overwrite
  
  for(std::vector<std::string>::const_iterator i=labels.begin(); i!=labels.end(); ++i)
    metadata_out << *i << ",";
  metadata_out << "OUT";
  
  metadata_out.close();
  
  ROS_DEBUG_STREAM("metadata: created in " << metadata_path);
  return true;
}

//! Bla bla..
/*!
  requires data_collector.hpp
*/
bool
get_samples(const std::vector<std::string>& tmm_paths)
{
  return false;
//  ROS_DEBUG_STREAM("tmm_paths.size()= " << tmm_paths.size());
//  for(std::vector<std::string>::const_iterator i= tmm_paths.begin(); i!=tmm_paths.end(); ++i)
//  {
//    // Read the validated(geometrically-planned) tmm
//    TaskMotionMultigraph tmm;
//    boost::dynamic_properties tmm_dp;
//    
//    tmm_dp.property("vertex_id", get(vertex_name, tmm));
//    
//    tmm_dp.property("label", get(edge_name, tmm));
//    tmm_dp.property("weight", get(edge_weight, tmm));
//    tmm_dp.property("jspace", get(edge_jspace, tmm)); 
//    tmm_dp.property("color", get(edge_color,tmm));
//    tmm_dp.property("srcstate", get(edge_srcstate,tmm));
//    tmm_dp.property("mptime",get(edge_mptime,tmm));
//    tmm_dp.property("planstr",get(edge_planstr,tmm));
//        
//    std::ifstream tmm_dot(i->c_str());
//    if( !read_graphviz(tmm_dot, tmm, tmm_dp, "vertex_id") )
//    {
//      ROS_ERROR("read_graphviz() failed.");
//      return false;
//    }
//    
//    // Remove more-expensive edges, remove parallelism
//    std::set<TMMEdge> tobe_removed_edges;
//    
//    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
//    for(boost::tie(vi,vi_end) = vertices(tmm); vi!=vi_end; ++vi)
//    {
//      std::map<TMMVertex,TMMEdge> tv_e_map;
//      
//      graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
//      for(tie(oei,oei_end) = out_edges(*vi,tmm); oei!=oei_end; ++oei )
//      {
//        std::map<TMMVertex,TMMEdge>::iterator it;
//        bool inserted;
//        
//        tie(it,inserted) = tv_e_map.insert( std::make_pair(target(*oei,tmm),*oei) );
//        
//        if(!inserted)
//        {
//          if(get(edge_weight,tmm,*oei) < get(edge_weight,tmm,it->second))
//          {
//            tobe_removed_edges.insert(it->second);

//            it->second = *oei;
//          }
//          else
//          {
//            tobe_removed_edges.insert(*oei);
//          }
//        }
//      }
//    }// end of: For each vertex in tmm
//    
//    for(std::set<TMMEdge>::const_iterator i=tobe_removed_edges.begin(); i!=tobe_removed_edges.end(); ++i)
//      remove_edge(*i,tmm);
//    ROS_DEBUG("Parallelism: removed");
//    
//    // Filter only the validated(geometrically-planned) edge to make dfs_visit more efficient by cutting the depth of the tmm
//    PlannedEdgeFilter<TMMEdgeColorMap> planned_edge_filter( get(edge_color, tmm) );
//    typedef filtered_graph< TaskMotionMultigraph, PlannedEdgeFilter<TMMEdgeColorMap> > PlannedTMM;

//    PlannedTMM planned_only_tmm(tmm, planned_edge_filter);
//    ROS_DEBUG_STREAM("num_vertices(planned_only_tmm)= " << num_vertices(planned_only_tmm));
//    ROS_DEBUG_STREAM("num_edges(planned_only_tmm)= " << num_edges(planned_only_tmm));
//    
////    // Write the filtered tmm: planned_only_tmm
////    boost::dynamic_properties planned_only_tmm_dp;
////  
////    planned_only_tmm_dp.property( "vertex_id",get(vertex_name,planned_only_tmm) );
////    planned_only_tmm_dp.property( "label",get(edge_name, planned_only_tmm) );
////    planned_only_tmm_dp.property( "weight",get(edge_weight, planned_only_tmm) );
////    planned_only_tmm_dp.property( "jspace",get(edge_jspace, planned_only_tmm) );
////    planned_only_tmm_dp.property( "color",get(edge_color, planned_only_tmm) );
////    planned_only_tmm_dp.property( "srcstate",get(edge_srcstate,planned_only_tmm) );

////    std::string planned_only_tmm_dot_path;
////    planned_only_tmm_dot_path = data_path_ + "/planned_only_tmm.dot";
////    
////    ofstream planned_only_tmm_dot;
////    planned_only_tmm_dot.open(planned_only_tmm_dot_path.c_str());
////      
////    write_graphviz_dp( planned_only_tmm_dot, planned_only_tmm, planned_only_tmm_dp, std::string("vertex_id"));
////    planned_only_tmm_dot.close();
//    
//    // Do dfs to get planned paths 
//    // and extract features from them using DataCollector as a dfs visitor
//    
//    PlannedTMM::vertex_descriptor root;
//    boost::graph_traits<PlannedTMM>::vertex_iterator vj, vj_end;
//    for(boost::tie(vj,vj_end) = vertices(planned_only_tmm); vj!=vj_end; ++vj)
//    {
//      if( !strcmp(get(vertex_name,planned_only_tmm,*vj).c_str(),"MessyHome") )
//        root = *vj;
//    }
//    
//    std::string metadata_path;
//    metadata_path = data_path_+"/ml_data/metadata.csv";
//    
//    // TODO should not it be <PlannedTMM> ?
//    data_collector::DataCollector<TaskMotionMultigraph> dc(&tr_data_,metadata_path);
//    
//    depth_first_visit( planned_only_tmm,root,dc,get(vertex_color,planned_only_tmm) );
//  }// End of: for each tmm_path
//  
//  // Write data to a csv file  
//  std::string tr_data_path;
//  tr_data_path = data_path_ + "/ml_data/data.csv";
//  
//  std::ofstream tr_data_out;
//  tr_data_out.open( tr_data_path.c_str() );// overwrite
//  
//  ROS_DEBUG_STREAM("tr_data_.size()= " << tr_data_.size());
//  for(Data::const_iterator i=tr_data_.begin(); i!=tr_data_.end(); ++i)
//  {
//    // Write input=feature values
//    for(Input::const_iterator j=i->first.begin(); j!=i->first.end(); ++j)
//    {
//      tr_data_out << *j << ",";
//    }
//    
//    tr_data_out << i->second << std::endl;
//  }
//  
//  ROS_DEBUG("Samples file: created");
//  tr_data_out.close();
//  
//  // Write data into a libsvm file
//  std::string tr_data_path_2;
//  tr_data_path_2 = data_path_ + "/ml_data/data.libsvmdata";
//  
//  data_util::write_libsvm_data(tr_data_,tr_data_path_2);
//  
//  return true;
}

//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

Data tr_data_;// so-called samples

std::string data_path_;
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "learning_machine");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  LearningMachine learner(nh);

  // $ rosservice call /create_metadata 5
  ros::ServiceServer create_metadata_srv;
  create_metadata_srv = nh.advertiseService("/create_metadata", &LearningMachine::create_metadata_srv_handle, &learner);
    
  ros::ServiceServer train_srv;
  train_srv = nh.advertiseService("/train", &LearningMachine::train_srv_handle, &learner);
  
  ROS_INFO("learner: spinning...");
  ros::spin();
  
  return 0;
}
