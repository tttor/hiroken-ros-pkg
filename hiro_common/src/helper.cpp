#include <ros/ros.h>

#include <arm_navigation_msgs/DisplayTrajectory.h>

#include <visualization_msgs/Marker.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include <vector>
#include <boost/math/constants/constants.hpp>
#include <math.h>
#include <algorithm>

#include "hiro_common/PathQuality.h"
#include "hiro_common/BenchmarkPath.h"
#include "hiro_common/GetManipulability.h"

#include "jacobian.hpp"
#include "hiro_utils.hpp"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

class Helper
{
public:
Helper(ros::NodeHandle nh):
  nh_(nh)
{
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  set_planning_scene_diff_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);
  
  display_filtered_path_pub_ = nh.advertise<arm_navigation_msgs::DisplayTrajectory>("display_filtered_path", 1, true);
  
  path_marker_pub_ = nh.advertise<visualization_msgs::Marker>("planned_path", 10);
  filtered_path_marker_pub_ = nh.advertise<visualization_msgs::Marker>("filtered_path", 10);
  path_quality_pub_ = nh.advertise<hiro_common::PathQuality>("path_quality", 10);
  
  collision_models_ = new planning_environment::CollisionModels("robot_description");
  planning_scene_state_ = NULL;
}

~Helper()
{
  delete[] collision_models_;// why does this: delete collision_models_; w/o [] seems to block the program
}

void
motion_plan_subs_cb(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  visualize_path(*msg);
//  visualize_motion_plan(*msg);
}

bool
benchmark_path_srv_handle(hiro_common::BenchmarkPath::Request& req, hiro_common::BenchmarkPath::Response& res)
{
  ROS_DEBUG("BenchmarkPath: BEGIN");
  
  if(req.trajectory.points.empty())
  {
    res.length = 0.;  
  }
  else
  {
    std::vector<geometry_msgs::Point> path;
    path = convert_to_cartesian(req.trajectory,std::string("/link_base"));
    
    // Length
    double len = 0.0;
    for (size_t i = 1 ; i < path.size() ; ++i)
        len += distance(path[i-1], path[i]);
        
    res.length = len;
  }
  
  ROS_DEBUG("BenchmarkPath: END");
  return true;
}

bool
get_manipulability_srv_handle(hiro_common::GetManipulability::Request& req,hiro_common::GetManipulability::Response& res)
{
  // Get a subset of joint states (that may contain the entire robot state) for given jspace
  planning_environment::CollisionModels collision_models("robot_description");
  
  std::vector<std::string> joint_names;
  joint_names = collision_models.getKinematicModel()->getModelGroup(req.jspace)->getJointModelNames();
  
  std::vector<double> joint_states( joint_names.size() );
  
  for(std::vector<std::string>::const_iterator i=joint_names.begin(); i!=joint_names.end(); ++i)
  {
    for(std::vector<std::string>::const_iterator j=req.jstate.name.begin(); j!=req.jstate.name.end(); ++j)
    {
      if( !strcmp(i->c_str(),j->c_str()) )
      {
        joint_states.at( i-joint_names.begin() ) = req.jstate.position.at( j-req.jstate.name.begin() );
        break;
      }
    }
  }  
  
  // Get the jacobian
  Jacobian jac_utils;
  jac_utils.initialize( get_base_link(req.jspace),get_tip_link(req.jspace) );
  
  Eigen::Map<Eigen::VectorXd> eigen_joint_states(&joint_states[0], joint_states.size());

//  Eigen::MatrixXd jacobian;
//  jac_utils.getJacobian(eigen_joint_states, jacobian);
  
  res.m = jac_utils.getManipulabilityMeasure(eigen_joint_states);
  ROS_DEBUG_STREAM("m= " << jac_utils.getManipulabilityMeasure(eigen_joint_states));

  return true;
}

private:
//! Visualize the motion plan
void
visualize_path(const trajectory_msgs::JointTrajectory& path)
{
  visualization_msgs::Marker edges;
  
  edges.header.frame_id = "/link_base";
  edges.header.stamp  = ros::Time::now();
  edges.ns = "filtered_trajectory";
  edges.action = visualization_msgs::Marker::ADD;
  edges.type = visualization_msgs::Marker::LINE_STRIP;
  edges.id = 1;
  edges.scale.x = 0.03;
  edges.color.g = 1.0;
  edges.color.a = 1.0;
  edges.pose.orientation.w = 1.0;
  
  // Obtain the jspace
  std::string jspace;
  jspace = get_jspace(path.joint_names);
  ROS_DEBUG_STREAM("jspace= " << jspace);
  ROS_DEBUG_STREAM("tip_link= " << get_tip_link(jspace));
  
  if( path.points.empty() )
  { 
    edges.action = visualization_msgs::Marker::DELETE;
    filtered_path_marker_pub_.publish(edges);
    return;
  }
    
  for(size_t i=0; i < path.points.size(); ++i)
  {
    kinematics_msgs::GetPositionFK::Response fk_response;
    fk_response = solve_fk(path.points.at(i),jspace,edges.header.frame_id,get_tip_link(jspace));
    
    //ROS_INFO_STREAM("fk_response.pose_stamped.size= " << fk_response.pose_stamped.size());
    // For now, there actually is only 1 stamp 
    for(size_t j=0; j< fk_response.pose_stamped.size(); ++j)
    {
    /*
      ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[j].c_str());
      ROS_INFO_STREAM("Position: " << 
        fk_response.pose_stamped[j].pose.position.x << "," <<  
        fk_response.pose_stamped[j].pose.position.y << "," << 
        fk_response.pose_stamped[j].pose.position.z);
      ROS_INFO("Orientation: %f %f %f %f",
        fk_response.pose_stamped[j].pose.orientation.x,
        fk_response.pose_stamped[j].pose.orientation.y,
        fk_response.pose_stamped[j].pose.orientation.z,
        fk_response.pose_stamped[j].pose.orientation.w);
    */
      geometry_msgs::Point p;
      p.x = fk_response.pose_stamped[j].pose.position.x;
      p.y = fk_response.pose_stamped[j].pose.position.y;
      p.z = fk_response.pose_stamped[j].pose.position.z;

      edges.points.push_back(p);
    } 
    filtered_path_marker_pub_.publish(edges);
  }// end of: for-each point in the path
}

std::vector<geometry_msgs::Point>
convert_to_cartesian(const trajectory_msgs::JointTrajectory& joint_trajectory,const std::string& ref_frame)
{
  // Obtain the jspace
  std::string jspace;
  jspace = get_jspace(joint_trajectory.joint_names);
    
  std::vector<geometry_msgs::Point> cartesian_trajectory;
  
  for(size_t i=0; i < joint_trajectory.points.size(); ++i)
  {
    kinematics_msgs::GetPositionFK::Response fk_response;
    fk_response = solve_fk(joint_trajectory.points.at(i),jspace,ref_frame,get_tip_link(jspace));
    
    //ROS_INFO_STREAM("fk_response.pose_stamped.size= " << fk_response.pose_stamped.size());
    // For now, there actually is only 1 stamp 
    for(size_t j=0; j< fk_response.pose_stamped.size(); ++j)
    {
    /*
      ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[j].c_str());
      ROS_INFO_STREAM("Position: " << 
        fk_response.pose_stamped[j].pose.position.x << "," <<  
        fk_response.pose_stamped[j].pose.position.y << "," << 
        fk_response.pose_stamped[j].pose.position.z);
      ROS_INFO("Orientation: %f %f %f %f",
        fk_response.pose_stamped[j].pose.orientation.x,
        fk_response.pose_stamped[j].pose.orientation.y,
        fk_response.pose_stamped[j].pose.orientation.z,
        fk_response.pose_stamped[j].pose.orientation.w);
    */
      geometry_msgs::Point p;
      p.x = fk_response.pose_stamped[j].pose.position.x;
      p.y = fk_response.pose_stamped[j].pose.position.y;
      p.z = fk_response.pose_stamped[j].pose.position.z;
      
      cartesian_trajectory.push_back(p);
    } 
  }
  return cartesian_trajectory;
}

void
benchmark_path(const std::vector<geometry_msgs::Point>& path)
{
  // Length
  double len = 0.0;
  for (size_t i = 1 ; i < path.size() ; ++i)
      len += distance(path[i-1], path[i]);
//  ROS_INFO_STREAM("path_len= " << len);    
  
  double s = 0.0;
  s = smoothness(path);
//  ROS_INFO_STREAM("path_s= " << s);    
  
  //TODO clearance()
  
  // Publish
  hiro_common::PathQuality path_quality;
  
  path_quality.length = len;
  path_quality.smoothness = s;
  
  path_quality_pub_.publish(path_quality);
}

double
distance(const geometry_msgs::Point& point_1, const geometry_msgs::Point& point_2)
{
  double dist = 0.0;
  
  //This is merely a Euclidean distance  
  dist = sqrt(  pow((point_1.x-point_2.x), 2) + pow((point_1.y-point_2.y), 2) + pow((point_1.z-point_2.z), 2)  );
  
  return dist;
}
double// TODO recheck me!
smoothness(const std::vector<geometry_msgs::Point>& path)
{
  double s = 0.0;
  if (path.size() > 2)
  {
      double a = distance(path[0], path[1]);
      for (size_t i = 2 ; i < path.size() ; ++i)
      {
          // view the path as a sequence of segments, and look at the triangles it forms:
          //          s1
          //          /\          s4
          //      a  /  \ b       |
          //        /    \        |
          //       /......\_______|
          //     s0    c   s2     s3
          //
          // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
          double b = distance(path[i-1], path[i]);
          double c = distance(path[i-2], path[i]);
          double acosValue = (a*a + b*b - c*c) / (2.0*a*b);

          if (acosValue > -1.0 && acosValue < 1.0)
          {
            // the smoothness is actually the outside angle of the one we compute
            double angle = (boost::math::constants::pi<double>() - acos(acosValue));
            
            // and we normalize by the length of the segments
            double k = 2.0 * angle / (a + b);
            s += k * k;
          }
          a = b;
      }
  }
  return s;
}

bool 
revertPlanningScene() 
{
  if(planning_scene_state_ != NULL) {
    collision_models_->revertPlanningScene(planning_scene_state_);
    planning_scene_state_ = NULL;
  }
  return true;
}

//! visualize motion plan with the whole body
void 
visualize_motion_plan(const trajectory_msgs::JointTrajectory& trajectory)
{
//  arm_navigation_msgs::DisplayTrajectory d_path;
//  
//  std::string group_name;
//  
//  if( !ros::param::get("/hiro_move_rarm/group", group_name) )
//  {
//    ROS_ERROR("Can not get /hiro_move_rarm/group");
//    return;
//  }
//  
//  d_path.model_id = group_name;
//  d_path.trajectory.joint_trajectory = trajectory;
//  
//  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
//  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
//  
//  revertPlanningScene();
//      
//  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
//    ROS_WARN("Can't get planning scene");
//    return;
//  }
//  
//  planning_scene_state_ = collision_models_->setPlanningScene(planning_scene_res.planning_scene);

//  planning_environment::setRobotStateAndComputeTransforms(planning_scene_res.planning_scene.robot_state, *planning_scene_state_);
//  
//  planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
//                                                          ros::Time::now(),
//                                                          collision_models_->getWorldFrameId(),
//                                                          d_path.robot_state);
//  display_filtered_path_pub_.publish(d_path);
}

kinematics_msgs::GetPositionFK::Response
solve_fk(const trajectory_msgs::JointTrajectoryPoint& point, const std::string& jspace, const std::string& ref_frame, const std::string& target_link)
{
  // Sanity checks: Get the info, ...
  std::string get_fk_solver_info_str = "hiro_" + jspace + "_kinematics_2/get_fk_solver_info";
  std::string get_fk_str = "hiro_" + jspace + "_kinematics_2/get_fk";
    
  ros::service::waitForService(get_fk_solver_info_str);
  ros::service::waitForService(get_fk_str);
  
  ros::ServiceClient get_fk_solver_info_client = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(get_fk_solver_info_str);
  ros::ServiceClient get_fk_client = nh_.serviceClient<kinematics_msgs::GetPositionFK>(get_fk_str);
  
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  
  if(get_fk_solver_info_client.call(request,response))
  {
//    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//      ROS_DEBUG("Joint: %d %s", i,response.kinematic_solver_info.joint_names[i].c_str());
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  
  // Get the real work: the fk solution
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  
  fk_request.header.frame_id = ref_frame;
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = target_link;
  
  fk_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  fk_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(size_t j =0; j < response.kinematic_solver_info.joint_names.size();j++)
  {
    fk_request.robot_state.joint_state.position.at(j) = point.positions.at(j);
  }
  
  if(get_fk_client.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val != fk_response.error_code.SUCCESS)
      ROS_ERROR("Forward kinematics failed");
  }
  else
    ROS_ERROR("Forward kinematics service call failed");

  return fk_response;
}

std::string
get_jspace(std::vector<std::string> jnames)
{
  std::string jspace;

  std::vector<std::string>::iterator it;
  it = std::find(jnames.begin(),jnames.end(),std::string("joint_rwrist_roll"));// the joint name to match to is arbitratily chosen
  
  ROS_DEBUG_STREAM("jnames.size()= " << jnames.size());
  
  if(it==jnames.end())//DOES NOT contains link_rhand_palm
  {
    if(jnames.size()==7)
    {
      jspace = "larm_U_chest";
    }
    else if(jnames.size()==6)
    {
      jspace = "larm";
    }  
  }
  else//contains link_rhand_palm
  {
    if(jnames.size()==7)
    {
      jspace = "rarm_U_chest";
    }
    else if(jnames.size()==6)
    {
      jspace = "rarm";
    }  
  }
  
  return jspace;
}

ros::NodeHandle nh_;

ros::Publisher path_marker_pub_;
ros::Publisher filtered_path_marker_pub_;
ros::Publisher display_filtered_path_pub_;
ros::Publisher path_quality_pub_;

ros::ServiceClient set_planning_scene_diff_client_;

planning_environment::CollisionModels* collision_models_;
planning_models::KinematicState* planning_scene_state_;
};//end of: class Helper

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "helper");
  ros::NodeHandle nh;

  Helper helper(nh);
  
//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  ros::Subscriber display_filtered_path_sub = nh.subscribe("rarm_controller/command", 1, &Helper::motion_plan_subs_cb, &helper);
  ros::Subscriber motion_plan_sub = nh.subscribe("motion_plan", 1, &Helper::motion_plan_subs_cb, &helper);
  
  ros::ServiceServer benchmark_path_srv;
  benchmark_path_srv = nh.advertiseService("benchmark_motion_plan", &Helper::benchmark_path_srv_handle, &helper);

  ros::ServiceServer get_manipulability_srv;
  get_manipulability_srv = nh.advertiseService("get_manipulability", &Helper::get_manipulability_srv_handle, &helper);
  
  // TODO !!!  
//  ros::ServiceServer display_motion_plan_srv_;
//  display_motion_plan_srv_ = nh.advertiseService("display_motion_plan", &TaskPlanner::display_motion_plan, &tp);
  
  ROS_INFO("helper:: UP and RUNNING");
  
  ros::Rate loop_rate(10);
  while( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
