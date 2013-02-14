#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>

#include <boost/algorithm/string/erase.hpp>
#include <fstream>

#include "ws_anal/EvalWS.h"
#include "hiro_utils.hpp"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

static const size_t N_POINTS_ON_SPHERE = 10;
static const size_t N_GRID_ONE_DIM = 25;//should be an odd number, otherwise it will be added by one
static const double SPHERE_RADIUS = 0.025;
static const double SPHERE_COLOR_A = 0.7;
static const double PITCH_STEP = M_PI/6.;// in rad, for rotating the frame of a point by which IK is computed

typedef geometry_msgs::Point Point;
typedef double Radius;
typedef std::pair<Point,Radius> Sphere;

struct Grid
{
  Grid()
  { 
    D = 101.;//for rank=0 (UNDEFINED)
  }
  
  Sphere sphere;
  std::vector<geometry_msgs::Point> points;
  
  double D;// the reachability index
};

typedef std::map<size_t,Grid> GridMap;

class WorkspaceAnalyzer
{
public:
WorkspaceAnalyzer(ros::NodeHandle& nh)
: nh_(nh)
{
  grid_pub_ = nh_.advertise<visualization_msgs::Marker>("grid_marker",1);
  state_validity_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);
}

bool
ws_eval_srv_handle(ws_anal::EvalWS::Request& req,ws_anal::EvalWS::Response& res)
{
  if( eval_grids(req.rbt_id,req.jspace,req.path) )
    res.msg = "Success";
  else
    res.msg = "Failed";
  return true;
}

void
vis_grids()
{
  ros::Rate r(10.);
  
  while( ros::ok() )
  {
    for(GridMap::const_iterator i=grid_map_.begin(); i!=grid_map_.end() and ros::ok(); ++i)
    {
      vis_grid(i->first,i->second);
      
      ros::spinOnce();
      r.sleep();
    }
  }
}
//! Evaluate points on a sphere grid
/*!
  Whether they have a valid IK
*/
bool
eval_grids(const std::string& rbt_id,const std::string& jspace,const std::string& path)
{
  ROS_DEBUG("eval_grids(): BEGIN");
  
  for(GridMap::iterator i=grid_map_.begin(); i!=grid_map_.end(); ++i)
  {
    ROS_DEBUG_STREAM("Evaluating grid ID= " << i->first  << " of " << grid_map_.size());
    
    size_t R = 0; // R is the number of valid inverse kinematics solutions recorded
    for(std::vector<geometry_msgs::Point>::const_iterator j=i->second.points.begin(); j!=i->second.points.end(); ++j)
    {
      if( eval_point(*j,i->second.sphere,rbt_id,jspace) )
        ++R;
      
      ROS_DEBUG_STREAM("Evaluated " << j-i->second.points.begin()+1  << " points of " << i->second.points.size());
    }
    
    i->second.D = (double)R / (double)i->second.points.size() * 100.;
    ROS_DEBUG_STREAM("D= " << i->second.D);
    
    // Update the grid
    vis_grid(i->first,i->second);
    
  }// end of: for each grid
  
  write_grids(path);
  
  ROS_DEBUG("eval_grids(): END");
  return true;
}
//! Build grid_map_
/*!
  Reference frame = /table 
  Grids are represented with spheres
  \param n_sphere the number of spheres along a dimension
*/
bool
build_grids(size_t n, const double radius,const std::string& path)
{
  // n must be odd
  if(!(n%2))
    n++;
  
  // The most negative sphere-center along x-axis
  Point c_ref;
  c_ref.x = (-1)*(2*radius)*(n/2);
  c_ref.y = (-1)*(2*radius)*(n/2);
  c_ref.z = 0.;// deliberately begins at z=0.
  
  size_t counter = 0;  
  for(size_t nx=0; nx<n; ++nx)// Sweep along x-axis
  {
    // Make a sphere
    Point center;
    center.x = c_ref.x + (nx*2*radius);
    
    for(size_t ny=0; ny<n; ++ny)// Sweep along y-axis
    { 
      center.y = c_ref.y+ (ny*2*radius);
      
      for(size_t nz=0; nz<n; ++nz)// Sweep along z-axis
      {
        center.z = c_ref.z+ (nz*2*radius);
        
        // Distribute points in this sphere
        std::vector<geometry_msgs::Point> points;
        saff_spiral(&points,N_POINTS_ON_SPHERE,center,radius);
        
        // Construct a grid
        Grid grid;
        grid.sphere = std::make_pair(center,radius);
        grid.points = points;
        
        grid_map_[counter] = grid;
        counter++;
      }
    }
  }
  
  write_grids(path);
  
  return true;
}

bool
build_grids(const std::string& path)
{
  std::ifstream gm_in(path.c_str());

  if(gm_in.is_open())
  {
    std::string line;
    while ( gm_in.good() )
    {
      getline(gm_in,line);
      
      std::vector<std::string> line_comps;
      boost::split( line_comps, line, boost::is_any_of("/") );
      
      if(line_comps.size()!=4)
        return false;
        
      std::string id_str = line_comps.at(0);
      std::string sphere_str = line_comps.at(1);
      std::string points_str = line_comps.at(2);
      std::string D_str = line_comps.at(3);
      
      // Construct a grid
      Grid grid;
      
      std::vector<std::string> sphere_str_comps;
      boost::split( sphere_str_comps,sphere_str,boost::is_any_of(",") );
      
      Point center;
      center.x = boost::lexical_cast<double>(sphere_str_comps.at(0));
      center.y = boost::lexical_cast<double>(sphere_str_comps.at(1));
      center.z = boost::lexical_cast<double>(sphere_str_comps.at(2));
      
      Radius radius = boost::lexical_cast<double>(sphere_str_comps.at(3));
      
      grid.sphere = std::make_pair(center,radius);
      
      std::vector<std::string> points_str_comps;
      boost::split( points_str_comps,points_str,boost::is_any_of(";") );
      points_str_comps.erase( points_str_comps.end()-1 );
      
      std::vector<Point> points;
      for(std::vector<std::string>::const_iterator i=points_str_comps.begin(); i!=points_str_comps.end(); ++i)
      {
        std::vector<std::string> i_comps;
        boost::split( i_comps,*i,boost::is_any_of(",") );
        
        Point p;
        p.x = boost::lexical_cast<double>(i_comps.at(0));
        p.y = boost::lexical_cast<double>(i_comps.at(1));
        p.z = boost::lexical_cast<double>(i_comps.at(2));
        
        points.push_back(p);
      }
      grid.points = points; 
      
      // The D
      grid.D = boost::lexical_cast<double>(D_str);
      
      // Push into the map
      grid_map_[boost::lexical_cast<double>(id_str)] = grid;
    }
    gm_in.close();
  }
  else
  {
   ROS_ERROR("Unable to open the foo.grid file");
   return false;
  }
  
  return true;
}

private:
//! Eval a point
/*
  http://demonstrations.wolfram.com/TangentPlaneToASphere/
  
  how to get 3 non-collinear points on a plane?
  http://stackoverflow.com/questions/8387199/how-do-i-get-three-non-colinear-points-on-a-plane-c
  
  http://answers.ros.org/question/44250/how-do-you-convert-a-quaternion-into-an-axis-angle-representation/
*/
bool
eval_point(const geometry_msgs::Point& p_wrt_table_frame,const Sphere& s,const std::string& rbt_id,const std::string& jspace)
{
  Point c = s.first;// the sphere center is described in /table
  double radius = s.second;
  
  const size_t steps = (2.*M_PI)/PITCH_STEP;
  
  static tf::TransformBroadcaster tf_bc;
  ros::Rate rate(10.);
  for(size_t i=0; i<steps; ++i)
  {
    // the rotation matrix should be based on /sphere
    // Therefore, p should be described in /sphere
    // Translation for /sphere to /table is in c (the sphere's center)
    tf::Transform sphere_frame;
    sphere_frame.setOrigin( tf::Vector3(c.x,c.y,c.z) );
    sphere_frame.setRotation( tf::createIdentityQuaternion() );
    tf_bc.sendTransform( tf::StampedTransform(sphere_frame,ros::Time::now(),"/table","/sphere") );
    
    Point p;
    p.x = p_wrt_table_frame.x - c.x;// the sphere center is described in /sphere now
    p.y = p_wrt_table_frame.y - c.y;// the sphere center is described in /sphere now
    p.z = p_wrt_table_frame.z - c.z;// the sphere center is described in /sphere now        
  
    //TODO make naming below nicer!
    // Build the tf for end-effector (/link_Xhand_palm)
    double theta = acos(p.z/radius);//the inclination (or polar angle) is the angle between the zenith direction and the line segment OP.
    double phi = atan2(p.y,p.x);//the azimuth (or azimuthal angle) is the signed angle measured from the azimuth reference direction to the orthogonal projection of the line segment OP on the reference plane.
        
    // rotate/tf_B around Z-axis-of-/table to align X-axis-of-/B with the projection of R
    tf::Transform tf_B;
    tf_B = tf::Transform( tf::createQuaternionFromRPY(0, 0.,phi),tf::Vector3(0.,0.,0.) );
//    tf_bc.sendTransform( tf::StampedTransform(tf_B, ros::Time::now(), "/table", "/tf_B") );
    
    // rotate /tf_C around Y-axis-of-/tf_B to align Z-axis-of-/C with R
    tf::Transform tf_C;
    tf_C = tf::Transform( tf::createQuaternionFromRPY(0, theta,0.),tf::Vector3(0.,0.,0.) );
//    tf_bc.sendTransform( tf::StampedTransform(tf_C, ros::Time::now(), "/tf_B", "/tf_C") );
    
    // rotate to adapt hiro-hand frame
    tf::Transform tf_D;
    tf_D = tf::Transform( tf::createQuaternionFromRPY(-1*M_PI/2, 0.,0.),tf::Vector3(0.,0.,0.) );
//    tf_bc.sendTransform( tf::StampedTransform(tf_D, ros::Time::now(), "/tf_C", "/tf_D") );
    
    // to rotate around Y-axis (pitch) for varying grasp angle
    tf::Transform tf_E;
    tf_E = tf::Transform( tf::createQuaternionFromRPY(0.,i*PITCH_STEP,0.),tf::Vector3(0.,0.,0.) );
//    tf_bc.sendTransform( tf::StampedTransform(tf_E, ros::Time::now(), "/D", "/tf_E") );
    
    // This is only rotation transformation
    tf::Transform tf_rot_E_wrt_A;
    tf_rot_E_wrt_A = tf_B * tf_C * tf_D * tf_E;
//    tf_bc.sendTransform( tf::StampedTransform(tf_rot_D_wrt_A, ros::Time::now(), "/table", "/tf_F") );

    // the reference frame /table is already imposed below for assert_ik()
    // eof_tf here is coincident with /link_Xhand_palm
    tf::Transform eof_tf;
    eof_tf = tf::Transform( tf_rot_E_wrt_A.getRotation(),tf::Vector3(p.x,p.y,p.z) );
    tf_bc.sendTransform( tf::StampedTransform(eof_tf, ros::Time::now(), "/sphere", "/eof_tf") );
    
    // Check IK
    std::string ref_frame = "/sphere";// for IK solver, must be the same as /eof_tf reference frame
    if( assert_ik(eof_tf,ref_frame,rbt_id,jspace) )
    {
      ROS_DEBUG_STREAM("IK sol is found at step= " << i+1 << " of " << steps);
      return true;
    }
    rate.sleep();
  }
  
  return false;
}

bool
write_grids(const std::string& path)
{
  std::ofstream gm_out;// stands for grid_map
  gm_out.open(path.c_str());
  
  for(GridMap::const_iterator i=grid_map_.begin(); i!=grid_map_.end(); ++i)
  {
    gm_out << i->first << "/";// ID
    gm_out << i->second.sphere.first.x << "," << i->second.sphere.first.y << "," << i->second.sphere.first.z << "," << i->second.sphere.second << "/";// sphere
    
    // points
    for(std::vector<geometry_msgs::Point>::const_iterator j=i->second.points.begin(); j!=i->second.points.end(); ++j)
    {
      gm_out << j->x << "," << j->y << "," << j->z << ";";
    }
    gm_out << "/";
    
    // D
    gm_out << i->second.D;
    
    gm_out << std::endl;
  }
  
  gm_out.close();
  return true;
}

bool
assert_ik(const tf::Transform& eof_tf,const std::string ref_frame,const std::string& rbt_id,const std::string& jspace)
{
  // Obtain IK solver info
  std::string get_ik_solver_info_str = "hiro_" + jspace + "_kinematics_2/get_ik_solver_info";
  ros::service::waitForService(get_ik_solver_info_str);
  ros::ServiceClient ik_solver_info_client = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(get_ik_solver_info_str);

  kinematics_msgs::GetKinematicSolverInfo::Request gksi_req;
  kinematics_msgs::GetKinematicSolverInfo::Response gksi_res;
  if( !ik_solver_info_client.call(gksi_req, gksi_res) )
  {
    ROS_ERROR("Could not call IK info query service");
    return false;
  }
  
  // Run the IK solver
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;

  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = get_eof_link(rbt_id);
  gpik_req.ik_request.pose_stamped.header.frame_id = ref_frame;

  gpik_req.ik_request.pose_stamped.pose.position.x = eof_tf.getOrigin().x();
  gpik_req.ik_request.pose_stamped.pose.position.y = eof_tf.getOrigin().y();  
  gpik_req.ik_request.pose_stamped.pose.position.z = eof_tf.getOrigin().z();  
  gpik_req.ik_request.pose_stamped.pose.orientation.x = eof_tf.getRotation().x();
  gpik_req.ik_request.pose_stamped.pose.orientation.y = eof_tf.getRotation().y();
  gpik_req.ik_request.pose_stamped.pose.orientation.z = eof_tf.getRotation().z();
  gpik_req.ik_request.pose_stamped.pose.orientation.w = eof_tf.getRotation().w();

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(gksi_res.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = gksi_res.kinematic_solver_info.joint_names;
  
  for(size_t j=0; j<gksi_res.kinematic_solver_info.joint_names.size(); ++j)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[j] = (gksi_res.kinematic_solver_info.limits[j].min_position + gksi_res.kinematic_solver_info.limits[j].max_position)/2.0;
  }

  std::string get_ik_solver_str = "hiro_" + jspace + "_kinematics_2/get_ik";
  ros::service::waitForService(get_ik_solver_str);
  ros::ServiceClient ik_solver_client = nh_.serviceClient<kinematics_msgs::GetPositionIK>(get_ik_solver_str);
  
  if( !ik_solver_client.call(gpik_req, gpik_res) )
  {
    ROS_ERROR("Could not call IK solver service");
    return false;
  }
  
  if(gpik_res.error_code.val != gpik_res.error_code.SUCCESS)
  {
//    ROS_DEBUG("gpik_res.error_code.val != gpik_res.error_code.SUCCESS");
    return false;
  }
  
  // Further check whether in collision with obstacles
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient get_planning_scene_client;
  get_planning_scene_client = nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) 
  {
    ROS_ERROR("Can't get planning scene");
    return false;
  }

  planning_environment::CollisionModels collision_models("robot_description");

  planning_models::KinematicState* state = collision_models.setPlanningScene(planning_scene_res.planning_scene);
  
  std::vector<std::string> arm_names = collision_models.getKinematicModel()->getModelGroup(jspace)->getUpdatedLinkModelNames();
  std::vector<std::string> joint_names = collision_models.getKinematicModel()->getModelGroup(jspace)->getJointModelNames();
  
  std::map<std::string, double> joint_values;
  for(size_t k=0; k<joint_names.size(); ++k)
  {
   joint_values[joint_names.at(k)] = gpik_res.solution.joint_state.position.at(k);
  }
  state->setKinematicState(joint_values);
  
  std_msgs::ColorRGBA good_color, collision_color, joint_limits_color;
  good_color.a = collision_color.a = joint_limits_color.a = .8;

  good_color.g = 1.0;
  collision_color.r = 1.0;
  joint_limits_color.b = 1.0;
  
  std_msgs::ColorRGBA point_markers;
  point_markers.a = 1.0;
  point_markers.r = 1.0;
  point_markers.g = .8;

  std_msgs::ColorRGBA color;
  visualization_msgs::MarkerArray arr;

  bool passed = false;  
  if(!state->areJointsWithinBounds(joint_names)) 
  {
    color = joint_limits_color;
    passed = false;
  }
  else if(collision_models.isKinematicStateInCollision(*state)) 
  {
    color = collision_color;
    collision_models.getAllCollisionPointMarkers(*state,
                                                 arr,
                                                 point_markers,
                                                 ros::Duration(0.2));
    passed = false;
  }
  else 
  {
    color = good_color;
    passed = true;
  }

  collision_models.getRobotMarkersGivenState(*state,
                                             arr,
                                             color,
                                             jspace,
                                             ros::Duration(0.2),
                                             &arm_names);
                                             
  state_validity_marker_array_pub_.publish(arr);

  collision_models.revertPlanningScene(state); 
  
  return passed;
}

void
vis_grid(const size_t& id,const Grid& grid)
{
  // Visualize the sphere ============================================================================
  visualization_msgs::Marker sphere;

  geometry_msgs::Point center = grid.sphere.first;
  double radius = grid.sphere.second;      

  sphere.header.frame_id = "/table";
  sphere.header.stamp = ros::Time::now();
  sphere.ns = get_ns( get_rank(grid.D) );
  sphere.id = id;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;

  // Set the pose of the sphere.  This is a full 6DOF pose relative to the frame/time specified in the header
  sphere.pose.position.x = center.x;
  sphere.pose.position.y = center.y;
  sphere.pose.position.z = center.z;
  sphere.pose.orientation.x = 0.0;
  sphere.pose.orientation.y = 0.0;
  sphere.pose.orientation.z = 0.0;
  sphere.pose.orientation.w = 1.0;

  // Set the scale of the sphere -- 1x1x1 here means 1m on a side
  sphere.scale.x = 2*radius;
  sphere.scale.y = 2*radius;
  sphere.scale.z = 2*radius;

  // Set the color -- be sure to set alpha to something non-zero!
  sphere.color = get_color( get_rank(grid.D) );

  sphere.lifetime = ros::Duration();

  // Publish the sphere
  grid_pub_.publish(sphere);
  
  // Visualize points on the sphere ============================================================================
  visualization_msgs::Marker points;
  
  points.header.frame_id = "/table";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  
  points.id = id;
  points.type = visualization_msgs::Marker::POINTS;
  
  const double scale = 0.005;
  points.scale.x = scale;
  points.scale.y = scale;
  
  points.color.r = 1.0f;
  points.color.a = 1.0;

  // Distribute points uniformly in a sphere
  points.points = grid.points;

  grid_pub_.publish(points);
}

std::string
get_ns(const size_t& rank)
{
  std::string ns;
  
  switch(rank)
  {
    case 1:
      ns = "sphere_1"; break;
    case 2:
      ns = "sphere_2"; break;
    default:
      ns = "sphere";
  }
  
  return ns;
}

std_msgs::ColorRGBA
get_color(const size_t& rank)
{
  std_msgs::ColorRGBA color;
  
  switch(rank)
  {
    case 1:
    {
      color.r = 0.;
      color.g = 1.;
      color.b = 0.;
      color.a = SPHERE_COLOR_A;
      
      break;
    }
    case 2:
    {
      color.r = 1.;
      color.g = 0.;
      color.b = 0.;
      color.a = SPHERE_COLOR_A;
      
      break;
    }
    default:
    {
      color.r = 0.;
      color.g = 0.;
      color.b = 1.;
      color.a = SPHERE_COLOR_A;
    }
  }
  
  return color;
}
//! Get rank
/*!
  BEST => rank=1
  UNDEFINED => rank=0
*/
size_t
get_rank(const double& D)
{
  if( (D>=90.)and(D<101.) )
    return 1;
  else if( (D>=0.)and(D<90.) )
    return 2;
  else
    return 0;
}

void
saff_spiral(std::vector<geometry_msgs::Point>* points,const size_t& n,const geometry_msgs::Point& center,double radius=1.0)
{
  const double C = 3.6;
  
  double pre_phi = 0.;// equals phi at k=1 and k=n
  for(size_t k=1; k<=n; ++k)
  {
    double h;
    h = -1. + ( (2.*(k-1))/(n-1) );
    
    double theta;
    theta = acos(h);
    
    double phi;
    if(k==1 or k==n)
      phi = 0.;
    else
      phi = (  pre_phi + ( (C/sqrt(n))*(1./sqrt(1.-pow(h,2))) )  );
      
    if(phi > (2.0*M_PI))
      phi = phi - (2.0*M_PI);
      
    pre_phi = phi;
    
    geometry_msgs::Point point;
    point.x = radius * sin(theta) * cos(phi);
    point.y = radius * sin(theta) * sin(phi);
    point.z = radius * cos(theta);
    
    // Transform w.r.t the center of the sphere
    point.x += center.x;
    point.y += center.y;
    point.z += center.z;
    
    points->push_back(point);
  }
}

ros::NodeHandle nh_;
GridMap grid_map_;
ros::Publisher grid_pub_;
ros::Publisher state_validity_marker_array_pub_;
};

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "ws_analyzer");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  WorkspaceAnalyzer ws_anal(nh);

  ros::ServiceServer ws_eval_srv;// must be outside the case{} block
        
  int mode = 0;
  if( !ros::param::get("/mode",mode) )
  ROS_WARN("Can not get /mode, use the default value (=0) instead");
  
  std::string grid_path; 
  if( !ros::param::get("/grid_path",grid_path) )
    ROS_WARN("Can not get /grid_path, use the default value instead");  

  switch(mode)
  {
    case 1:// BUILD then offer evaluation
    {
      ws_anal.build_grids(N_GRID_ONE_DIM,SPHERE_RADIUS,grid_path);
      
      ws_eval_srv = nh.advertiseService("eval_ws", &WorkspaceAnalyzer::ws_eval_srv_handle, &ws_anal); 
      
      break;
    }
    case 2:// MERELY re-build 
    {
      ws_anal.build_grids(grid_path);
      
      break;
    }
  }
  
  ws_anal.vis_grids();
  
  ROS_INFO("ws_anal: spinning...");
  ros::spin();
  
  ros::shutdown();
  return 0;
};
