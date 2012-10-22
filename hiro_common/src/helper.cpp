#include <ros/ros.h>

#include <arm_navigation_msgs/DisplayTrajectory.h>

#include <visualization_msgs/Marker.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include "hiro_common/PathQuality.h"
#include "hiro_common/BenchmarkPath.h"

#include <vector>
#include <boost/math/constants/constants.hpp>
#include <math.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

static const std::string LINK_BASE_R = "/link_base";
static const std::string LINK_TIP_R = "/link_rhand_palm";

class Helper
{
public:
Helper(ros::NodeHandle nh):
  nh_(nh)
{
  ros::service::waitForService("hiro_rarm_kinematics_2/get_fk_solver_info");
  ros::service::waitForService("hiro_rarm_kinematics_2/get_fk");
  
  get_fk_solver_info_client_ = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("hiro_rarm_kinematics_2/get_fk_solver_info");
  get_fk_client_ = nh.serviceClient<kinematics_msgs::GetPositionFK>("hiro_rarm_kinematics_2/get_fk");
  
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
display_path_cb(const  arm_navigation_msgs::DisplayTrajectory::ConstPtr& msg)
{
  visualization_msgs::Marker points;

  points.header.frame_id = "/link_base";
  points.header.stamp = ros::Time::now();
  points.ns = "trajectory";
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.id = 0;    
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  points.pose.orientation.w = 1.0;
  
  visualization_msgs::Marker edges;
    
  edges.header.frame_id = "/link_base";
  edges.header.stamp  = ros::Time::now();
  edges.ns = "trajectory";
  edges.action = visualization_msgs::Marker::ADD;
  edges.type = visualization_msgs::Marker::LINE_STRIP;
  edges.id = 1;
  edges.scale.x = 0.03;
  edges.color.b = 1.0;
  edges.color.a = 1.0;
  edges.pose.orientation.w = 1.0;
  
  ROS_INFO_STREAM("msg->trajectory.joint_trajectory.points.size()= " << msg->trajectory.joint_trajectory.points.size());
  for(size_t i=0; i < msg->trajectory.joint_trajectory.points.size(); ++i)
  {
    kinematics_msgs::GetPositionFK::Response fk_response;
    fk_response = solve_fk(msg->trajectory.joint_trajectory.points.at(i));
    
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

      points.points.push_back(p);
      edges.points.push_back(p);
    } 
    //path_marker_pub_.publish(points);
    path_marker_pub_.publish(edges);
  }
}
void
display_filtered_path_cb(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("filtered_trajectory_msg->trajectory.joint_trajectory.points.size()= " << msg->points.size());
  
 // Visualize the path of the plan     
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
  
  if( msg->points.empty() )
  { 
    edges.action = visualization_msgs::Marker::DELETE;
    filtered_path_marker_pub_.publish(edges);
    return;
  }

  for(size_t i=0; i < msg->points.size(); ++i)
  {
    kinematics_msgs::GetPositionFK::Response fk_response;
    fk_response = solve_fk(msg->points.at(i));
    
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
  }
  
  // Visualize the plan 
  visualizePlan(*msg);
}

std::vector<geometry_msgs::Point>
convert_to_cartesian(const trajectory_msgs::JointTrajectory& joint_trajectory)
{
  std::vector<geometry_msgs::Point> cartesian_trajectory;
  
  for(size_t i=0; i < joint_trajectory.points.size(); ++i)
  {
    kinematics_msgs::GetPositionFK::Response fk_response;
    fk_response = solve_fk(joint_trajectory.points.at(i));
    
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

bool
benchmark_path_srv_cb(hiro_common::BenchmarkPath::Request& req, hiro_common::BenchmarkPath::Response& res)
{
  if(req.trajectory.points.empty())
  {
    res.length = 0.;  
  }
  else
  {
    std::vector<geometry_msgs::Point> path;
    path = convert_to_cartesian(req.trajectory);
    
    // Length
    double len = 0.0;
    for (size_t i = 1 ; i < path.size() ; ++i)
        len += distance(path[i-1], path[i]);
        
    res.length = len;
  }
  
  return true;
}

private:
ros::NodeHandle nh_;

ros::Publisher path_marker_pub_;
ros::Publisher filtered_path_marker_pub_;
ros::Publisher display_filtered_path_pub_;
ros::Publisher path_quality_pub_;

ros::ServiceClient get_fk_solver_info_client_;
ros::ServiceClient get_fk_client_;

ros::ServiceClient set_planning_scene_diff_client_;

planning_environment::CollisionModels* collision_models_;
planning_models::KinematicState* planning_scene_state_;

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
void 
visualizePlan(const trajectory_msgs::JointTrajectory& trajectory)
{
  arm_navigation_msgs::DisplayTrajectory d_path;
  
  std::string group_name;
  
  if( !ros::param::get("/hiro_move_rarm/group", group_name) )
  {
    ROS_WARN("Can not get /hiro_move_rarm/group");
    return;
  }
  
  //d_path.model_id = "rarm";// TODO should be obtained from parameter server
  d_path.model_id = group_name;
  d_path.trajectory.joint_trajectory = trajectory;
  
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
  
  revertPlanningScene();
      
  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return;
  }
  
  planning_scene_state_ = collision_models_->setPlanningScene(planning_scene_res.planning_scene);

  planning_environment::setRobotStateAndComputeTransforms(planning_scene_res.planning_scene.robot_state, *planning_scene_state_);
  
  planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                          ros::Time::now(),
                                                          collision_models_->getWorldFrameId(),
                                                          d_path.robot_state);
  display_filtered_path_pub_.publish(d_path);
}

kinematics_msgs::GetPositionFK::Response
solve_fk(const trajectory_msgs::JointTrajectoryPoint& point){
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  
  if(get_fk_solver_info_client_.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s", i,
       response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  
  fk_request.header.frame_id = LINK_BASE_R;
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = LINK_TIP_R;
  
  fk_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  fk_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(size_t j =0; j < response.kinematic_solver_info.joint_names.size();j++)
  {
    fk_request.robot_state.joint_state.position.at(j) = point.positions.at(j);
  }
  
  if(get_fk_client_.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val != fk_response.error_code.SUCCESS)
      ROS_ERROR("Forward kinematics failed");
  }
  else
    ROS_ERROR("Forward kinematics service call failed");

  return fk_response;
}
};//end of: class Helper

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "helper");
  ros::NodeHandle nh;

  Helper helper(nh);
  
  ros::Subscriber display_path_sub = nh.subscribe("display_path", 1, &Helper::display_path_cb, &helper);
  ros::Subscriber display_filtered_path_sub = nh.subscribe("rarm_controller/command", 1, &Helper::display_filtered_path_cb, &helper);
  ros::Subscriber motion_plan_sub = nh.subscribe("motion_plan", 1, &Helper::display_filtered_path_cb, &helper);
  
  ros::ServiceServer benchmark_path_srv;
  benchmark_path_srv = nh.advertiseService("benchmark_motion_plan", &Helper::benchmark_path_srv_cb, &helper);

  // TODO !!!  
//  ros::ServiceServer display_motion_plan_srv_;
//  display_motion_plan_srv_ = nh.advertiseService("benchmark_motion_plan", &TaskPlanner::display_motion_plan, &tp);
  
  ROS_INFO("helper:: UP and RUNNING");
  
  ros::Rate loop_rate(10);
  while( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
