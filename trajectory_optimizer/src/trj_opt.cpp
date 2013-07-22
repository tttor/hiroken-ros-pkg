#include <ros/ros.h>
#include "tmm_utils.hpp"
#include "utils.hpp"
#include "data.hpp"
#include "data_collector_prj6.hpp"

#include <boost/filesystem.hpp>

static std::string g_data_basepath = "/home/vektor/prj6/data";

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
get_data(const std::string baseline_dir_path,const std::string& metadata_path,const std::string& data_path)
{
  std::vector<std::string> sol_tmm_paths;
  get_datasrc_path(baseline_dir_path,&sol_tmm_paths);
  
  Data data;
    
  // Collect data from all sol_tmm paths  
  dc_prj6::DataCollector dc(metadata_path);
  
  for(size_t i=0; i<sol_tmm_paths.size(); ++i)
  {
    std::cerr << "Extracting on ith= " << i+1 << " of " << sol_tmm_paths.size() << " path= " << sol_tmm_paths.at(i) << endl;
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
        
    std::ifstream sol_tmm_dot( sol_tmm_paths.at(i).c_str() );
    if( !read_graphviz(sol_tmm_dot, sol_tmm, sol_tmm_dp, "vertex_id") )
    {
      std::cerr << "read_graphviz(...): failed" << std::endl;
      return false;
    }

    // Extract features
    Input in;
    dc.get_fval(sol_tmm,&in);
    
    // Extract the expected cluster 
    Output out;
    dc.get_out(sol_tmm_paths.at(i),&out);
    
    // Unite
    data.push_back( std::make_pair(in,out) );
  }
  
  // Write the data
  data_util::write_csv_data(data,data_path);
  
  return true;
}

private:
bool
get_datasrc_path(const std::string& src_path,std::vector<std::string>* sol_tmm_paths)
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

//! ...
ros::NodeHandle nh_;

};

int 
main(int argc, char **argv)
{ 
  // Init
  ros::init(argc, argv, "traj_opt");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  std::string metadata_path;
  metadata_path = std::string(g_data_basepath+"/ml_data/metadata.csv");
  
  std::string data_path;
  data_path = std::string(g_data_basepath+"/ml_data/data.csv");  
  
  std::string baseline_dir_path;
  baseline_dir_path = std::string(g_data_basepath+"/baseline/");
  
  // Main works
  dc_prj6::create_metadata(metadata_path);
  
  TrajectoryOptimizer traj_opt(nh);
  traj_opt.get_data(baseline_dir_path,metadata_path,data_path);
  
  // Closure
  ROS_INFO("traj_opt: spinning...");
  ros::spin();
  
  return 0;
}
