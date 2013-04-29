#include <ros/ros.h>
#include "tmm_utils.hpp"
#include "utils.hpp"
#include "data.hpp"
#include "data_collector_prj6.hpp"

#include <boost/filesystem.hpp>

static std::string g_data_basepath = "/home/vektor/6.tpctran/data";

bool
get_datasrc_path(const std::string& src_path,std::list<std::string>* sol_tmm_paths)
{
  boost::filesystem::path dir_path(src_path);
  if ( !exists(dir_path) ) return false;

  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for ( boost::filesystem::directory_iterator itr( dir_path );itr != end_itr; ++itr )
  {
    if ( is_directory(itr->status()) )
    {
      sol_tmm_paths->push_back( std::string(  itr->path().string()+"/sol_tmm.dot"  ) );
    }
  }
  
  return true;
}

class
TrajectoryOptimizer
{
public:
TrajectoryOptimizer(ros::NodeHandle nh)
: nh_(nh)
{ }

~TrajectoryOptimizer()
{ }

bool
get_data(const std::list<std::string>& sol_tmm_paths,const std::string& metadata_path,const std::string& data_path)
{
  dc_prj6::DataCollector dc(metadata_path);
  
  // Collect data from all sol_tmm paths  
  for(std::list<std::string>::const_iterator i=sol_tmm_paths.begin(); i!=sol_tmm_paths.end(); ++i)
  {
    // Read the sol.tmm
    TaskMotionMultigraph sol_tmm;
    boost::dynamic_properties sol_tmm_dp;
    
    sol_tmm_dp.property("vertex_id", get(vertex_name, sol_tmm));
    
    sol_tmm_dp.property( "label",get(edge_name, sol_tmm) );
    sol_tmm_dp.property( "weight",get(edge_weight, sol_tmm) );  
    sol_tmm_dp.property( "jspace",get(edge_jspace, sol_tmm) );
    sol_tmm_dp.property( "color",get(edge_color, sol_tmm) );
    sol_tmm_dp.property( "srcstate",get(edge_srcstate, sol_tmm) );  
    sol_tmm_dp.property( "mptime",get(edge_mptime, sol_tmm) );  
    sol_tmm_dp.property( "planstr",get(edge_planstr, sol_tmm) );
        
    std::ifstream sol_tmm_dot(i->c_str());
    if( !read_graphviz(sol_tmm_dot, sol_tmm, sol_tmm_dp, "vertex_id") )
    {
      std::cerr << "read_graphviz(...): failed" << std::endl;
      return false;
    }

    // Extract features from the target-edge e 
    dc.get_fval(sol_tmm);
  }
  
  // Write the data
  dc.write(data_path);
  
  return true;
}

private:
//! ...
ros::NodeHandle nh_;

};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "traj_opt");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  std::string metadata_path;
  metadata_path = std::string(g_data_basepath+"/ml_data/metadata.csv");
  
  std::string data_path;
  data_path = std::string(g_data_basepath+"/ml_data/data.csv");  
  
  std::list<std::string> sol_tmm_paths;
  get_datasrc_path(std::string(g_data_basepath+"/baseline"),&sol_tmm_paths);
  
  dc_prj6::create_metadata(metadata_path);
  
  TrajectoryOptimizer traj_opt(nh);
  traj_opt.get_data(sol_tmm_paths,metadata_path,data_path);

  ROS_INFO("traj_opt: spinning...");
  ros::spin();
  
  return 0;
}
