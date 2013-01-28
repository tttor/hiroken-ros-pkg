#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>

#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include "hiro_control/ControlArm.h"
#include "hiro_control/CommitMovingArm.h"
#include "hiro_control/MoveArm.h"

#include <fstream>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

static const double JOINT_CONTROL_RATE = 120.;// Prev. values: 30.;60.; (1./0.6)(considering the trasient time in controlling joint that is about 0.6s) NOTE that the so-called transient time depends on the sppped

class HiroJointController
{
public:
HiroJointController(ros::NodeHandle nh):
  nh_(nh)
{
  joint_states_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state_cmd", 1);

  joint_states_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("rarm_controller/feedback_states", 1);
  
  set_planning_scene_diff_client_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);
  
  control_rarm_client_ = nh_.serviceClient<hiro_control::ControlArm>("control_rarm");
  
  approved_ = false;
  
  online_ = false;
  if( !ros::param::get("/online_", online_) )
    ROS_WARN("Can not get /online_, use the default value (=false) instead");
}
~HiroJointController()
{

}
void
joint_trajectory_sub_cb(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Received a trajectory from joint_action_server containing " << msg->points.size() << " waypoints");
  
  control_joint(*msg);
}

bool
move_arm_srv_handle(hiro_control::MoveArm::Request& req, hiro_control::MoveArm::Response& res)
{
  ROS_DEBUG_STREAM("Received a trajectory containing " << req.trajectory.points.size() << " waypoints");
  
  control_joint(req.trajectory);
  
  return true;
}

bool
commit_moving_arm_srv_handle(hiro_control::CommitMovingArm::Request& req, hiro_control::CommitMovingArm::Response& res)
{
  switch(req.signal)
  {
    case 1:
            approved_ = true;
            res.msg = "Approval is received";
            break;
    default:
            approved_ = false;
            res.msg = "Do not know the signal";
  }
  return true;
}

private:
void
control_joint(const trajectory_msgs::JointTrajectory& path)
{
    // An approval mechanism for real execution
  ros::Rate waiting_rate(30.);
  while( !approved_ and online_ and ros::ok() )
  {
    ROS_INFO("Waiting the approval for execution, this gonna be real!");
    ros::spinOnce();
    waiting_rate.sleep();
  }
  approved_ = false;
  
  ROS_DEBUG_STREAM("Start controlling joints for " << path.points.size() << " waypoints");
  
  const size_t lucky_waypoint = 1;
  size_t n_lucky_waypoint = 0;
  
  // Publish joint_state_feedback, close the control loop;  
  control_msgs::FollowJointTrajectoryFeedback joint_state_feedback;

  joint_state_feedback.header.stamp = ros::Time::now();
  joint_state_feedback.joint_names.resize(path.joint_names.size());
  joint_state_feedback.desired.positions.resize(path.joint_names.size());
  joint_state_feedback.actual.positions.resize(path.joint_names.size());
  joint_state_feedback.error.positions.resize(path.joint_names.size());
          
  ros::Rate joint_control_rate(JOINT_CONTROL_RATE);// NOTE this is likely to affect only in offline mode
  
  for(size_t i=0; i<path.points.size();++i)
  {
    // For debugging
    ros::Duration control_duration;
    ros::Duration in_loop_duration;
    ros::Time in_loop_begin = ros::Time::now();
    
    // (Again) filter waypoints
    if(  ( (i%lucky_waypoint)!=0 ) and ( i!=(path.points.size()-1) )  )
      continue;
    else
      ++n_lucky_waypoint;

    if(online_)
    {
      // Send joint_state_cmd to control the real robot
      hiro_control::ControlArm::Request control_rarm_req;
      hiro_control::ControlArm::Response control_rarm_res;
      
      for(size_t j=0; j<path.joint_names.size();++j)
      {
        control_rarm_req.desired.positions.push_back( path.points.at(i).positions.at(j) );
        control_rarm_req.desired.velocities.push_back( path.points.at(i).velocities.at(j) );
        control_rarm_req.desired.accelerations.push_back( 0. );
        control_rarm_req.desired.time_from_start = ros::Duration(0.5);// Useless so far!
      }

      ROS_DEBUG("ros::service::waitForService:control_rarm");
      ros::service::waitForService("control_rarm");
      
      ros::Time begin = ros::Time::now();
      
      if( !control_rarm_client_.call(control_rarm_req, control_rarm_res) ) 
        ROS_WARN("Can't call control_rarm srv");
        
      control_duration = ros::Time::now() - begin;
      
      for(size_t j=0; j<path.joint_names.size();++j)
      {
        joint_state_feedback.joint_names.at(j) = path.joint_names.at(j);
        joint_state_feedback.desired.positions.at(j) = path.points.at(i).positions.at(j);
        joint_state_feedback.actual.positions.at(j) = control_rarm_res.actual.positions.at(j);
        joint_state_feedback.error.positions.at(j) = 0.;
      }
    }
    else // !online_
    {
      // Publish joint_state_cmd to simulated joints
      sensor_msgs::JointState joint_state_cmd;
    
      joint_state_cmd.header.stamp = ros::Time::now();  
      joint_state_cmd.name.resize(path.joint_names.size());
      joint_state_cmd.position.resize(path.joint_names.size());
      joint_state_cmd.velocity.resize(path.joint_names.size());
      joint_state_cmd.effort.resize(path.joint_names.size());
      
      for(size_t j=0; j<joint_state_cmd.name.size();++j)
      {
        joint_state_cmd.name[j] = path.joint_names.at(j);
        joint_state_cmd.position[j] = path.points.at(i).positions.at(j);
        joint_state_cmd.velocity[j] = 0.;
        joint_state_cmd.effort[j] = 0.;
      }
      
      joint_states_cmd_pub_.publish(joint_state_cmd);
      
      for(size_t j=0; j<path.joint_names.size();++j)
      {
        joint_state_feedback.joint_names.at(j) = path.joint_names.at(j);
        joint_state_feedback.desired.positions.at(j) = path.points.at(i).positions.at(j);// Dummy
        joint_state_feedback.actual.positions.at(j) = path.points.at(i).positions.at(j);// Dummy: as same as the request
        joint_state_feedback.error.positions.at(j) = 0.;// Dummy: as same as the request
      }
    }
    
    // For feedback
    joint_states_feedback_pub_.publish(joint_state_feedback);
    
    // For visualization
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

    if( !set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res) ) 
      ROS_WARN("Can't get planning scene");
    
    if(!online_)// The real control_rarm srv already takes time in online mode
      joint_control_rate.sleep();

    in_loop_duration = ros::Time::now() - in_loop_begin;
    
    ROS_DEBUG_STREAM( "Control Duration= "<< control_duration.toSec() );
    ROS_DEBUG_STREAM( "In loop Duration= "<< in_loop_duration.toSec() );
  }// end of: for(size_t i=0; i<path.points.size();++i)  

  ROS_DEBUG_STREAM( "Number of lucky_waypoint= " << n_lucky_waypoint << " of " <<  path.points.size() );
  ROS_DEBUG("Finish controlling joints for a trajectory");
}

ros::NodeHandle nh_;

ros::Publisher joint_states_cmd_pub_;
ros::Publisher joint_states_feedback_pub_;

ros::ServiceClient set_planning_scene_diff_client_;

ros::ServiceClient control_rarm_client_;

bool approved_;

bool online_;
};// end of: class HiroJointController

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "hiro_joint_ctrl");
  ros::NodeHandle nh;

//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  HiroJointController hjc(nh);
  
//  ros::Subscriber joint_trajectory_sub = nh.subscribe("rarm_controller/command", 1, &HiroJointController::joint_trajectory_sub_cb, &hjc);
//  
//  ros::ServiceServer commit_srv = nh.advertiseService("commit_moving_arm", &HiroJointController::commit_moving_arm_srv_handle, &hjc);
  
  ros::ServiceServer move_arm_srv = nh.advertiseService("/move_arm", &HiroJointController::move_arm_srv_handle, &hjc);
  
  ROS_INFO("hiro_joint_controller:: UP and RUNNING");

  ros::Rate loop_rate(30);
  while( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

