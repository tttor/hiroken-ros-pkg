#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>

#include <visualization_msgs/Marker.h>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>

#include <sensor_msgs/JointState.h>

#include "grasp_planner/PlanGrasp.h"

#include "hiro_utils.hpp"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const double GRASP_PADDING = 0.070;
static const size_t YAW_STEP = (360./5.);// It means each step is (2*M_PI)/YAW_STEP radian = (360)/YAW_STEP degree

class GraspPlanner
{
public:
GraspPlanner(ros::NodeHandle& nh)
: nh_(nh)
{
  state_validity_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);
}

~GraspPlanner()
{ }

bool
plan_grasp_srv_handle(grasp_planner::PlanGrasp::Request &req, grasp_planner::PlanGrasp::Response &res)
{
  ROS_DEBUG_STREAM("Grasp planning: BEGIN for " << req.object.id.c_str() << " in " << req.jspace );
  
  std::string get_ik_solver_info_str = "hiro_" + req.jspace + "_kinematics_2/get_ik_solver_info";
  ros::service::waitForService(get_ik_solver_info_str);
  
  ros::ServiceClient ik_solver_info_client = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(get_ik_solver_info_str);

  kinematics_msgs::GetKinematicSolverInfo::Request gksi_req;
  kinematics_msgs::GetKinematicSolverInfo::Response gksi_res;

  if( !ik_solver_info_client.call(gksi_req, gksi_res) )
  {
    ROS_ERROR("Could not call IK info query service");
    return false;
  }
   
  const size_t yaw_step = YAW_STEP;
  const double yaw_step_ang = (2*M_PI)/yaw_step;
  size_t num_grasp_plan = 0;
    
  //  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_bc;// Must be outside the loop where it is used
  
  ROS_DEBUG("Start looping over yaw_steps");
  ros::Rate rate(120.);// Should be higher that object_tf publishing rate (from vision_sensor))
  for(size_t i=0; i<yaw_step;++i)
  {
//    tf::StampedTransform object_tf;
//    
//    try
//    {
//      tf_listener.waitForTransform("/link_base", req.object.id.c_str(), ros::Time::now(), ros::Duration(3.0));
//      tf_listener.lookupTransform("/link_base", req.object.id.c_str(), ros::Time(0), object_tf);
//    }
//    catch (tf::TransformException ex)
//    {
//      ROS_ERROR("%s",ex.what());
//    }
    
    tf::Transform object_tf;
    object_tf = tf::Transform(  tf::Quaternion(req.object.poses.at(0).orientation.x, req.object.poses.at(0).orientation.y, req.object.poses.at(0).orientation.z, req.object.poses.at(0).orientation.w), 
                                tf::Vector3(req.object.poses.at(0).position.x, req.object.poses.at(0).position.y, req.object.poses.at(0).position.z)
                             );
    
    tf::Transform grasp_helper_1_tf;
    grasp_helper_1_tf = tf::Transform(tf::createQuaternionFromRPY(0., M_PI, i*yaw_step_ang), tf::Vector3(0., 0., 0.));  
    
    tf::Transform grasp_helper_2_tf;
    grasp_helper_2_tf = tf::Transform(tf::createQuaternionFromRPY(0., 0., 0.), tf::Vector3(0., -1*GRASP_PADDING, 0.));
    
    tf::Transform grasp_tf;        
    grasp_tf = object_tf * grasp_helper_1_tf * grasp_helper_2_tf;
    
    tf_bc.sendTransform( tf::StampedTransform(grasp_tf, ros::Time::now(), "/table", "/grasp") );
    
    // Check the IK
    kinematics_msgs::GetPositionIK::Request  gpik_req;
    kinematics_msgs::GetPositionIK::Response gpik_res;
  
    gpik_req.timeout = ros::Duration(5.0);
    gpik_req.ik_request.ik_link_name = get_eof_link(req.rbt_id);
    gpik_req.ik_request.pose_stamped.header.frame_id = "table";

    gpik_req.ik_request.pose_stamped.pose.position.x = grasp_tf.getOrigin().x();
    gpik_req.ik_request.pose_stamped.pose.position.y = grasp_tf.getOrigin().y();  
    gpik_req.ik_request.pose_stamped.pose.position.z = grasp_tf.getOrigin().z();  
    gpik_req.ik_request.pose_stamped.pose.orientation.x = grasp_tf.getRotation().x();
    gpik_req.ik_request.pose_stamped.pose.orientation.y = grasp_tf.getRotation().y();
    gpik_req.ik_request.pose_stamped.pose.orientation.z = grasp_tf.getRotation().z();
    gpik_req.ik_request.pose_stamped.pose.orientation.w = grasp_tf.getRotation().w();
  
    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(gksi_res.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = gksi_res.kinematic_solver_info.joint_names;
    
    for(size_t j=0; j<gksi_res.kinematic_solver_info.joint_names.size(); ++j)
    {
      gpik_req.ik_request.ik_seed_state.joint_state.position[j] = (gksi_res.kinematic_solver_info.limits[j].min_position + gksi_res.kinematic_solver_info.limits[j].max_position)/2.0;
    }
  
    std::string get_ik_solver_str = "hiro_" + req.jspace + "_kinematics_2/get_ik";
    ros::service::waitForService(get_ik_solver_str);
    
    ros::ServiceClient ik_solver_client = nh_.serviceClient<kinematics_msgs::GetPositionIK>(get_ik_solver_str);
    
    if(ik_solver_client.call(gpik_req, gpik_res))
    {
      if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      {
        // Check further
        ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
        get_planning_scene_client_ = nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

        arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
        arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

        if(!get_planning_scene_client_.call(planning_scene_req, planning_scene_res)) {
          ROS_WARN("Can't get planning scene");
          return -1;
        }

        planning_environment::CollisionModels collision_models("robot_description");

        planning_models::KinematicState* state = collision_models.setPlanningScene(planning_scene_res.planning_scene);
        
        std::vector<std::string> arm_names = collision_models.getKinematicModel()->getModelGroup(req.jspace)->getUpdatedLinkModelNames();
        std::vector<std::string> joint_names = collision_models.getKinematicModel()->getModelGroup(req.jspace)->getJointModelNames();
        
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
        
        if(!state->areJointsWithinBounds(joint_names)) 
        {
          color = joint_limits_color;
        }
        else if(collision_models.isKinematicStateInCollision(*state)) 
        {
          color = collision_color;
          collision_models.getAllCollisionPointMarkers(*state,
                                                       arr,
                                                       point_markers,
                                                       ros::Duration(0.2));
        }
        else 
        {
          color = good_color;
          
          ++num_grasp_plan;
          
          // Overwrite some stuff!
          gpik_res.solution.joint_state.header.seq = num_grasp_plan - 1;// The index begins at 0
          gpik_res.solution.joint_state.header.stamp = ros::Time::now();
          
          res.grasp_plans.push_back(gpik_res.solution.joint_state);
        }

        collision_models.getRobotMarkersGivenState(*state,
                                                   arr,
                                                   color,
                                                   req.jspace,
                                                   ros::Duration(0.2),
                                                   &arm_names);
        state_validity_marker_array_pub_.publish(arr);
        collision_models.revertPlanningScene(state); 
      }
      else
      {
        //ROS_ERROR("Inverse kinematics failed");
      }
    }
    else
    {
      //ROS_ERROR("Inverse kinematics service call failed");
    }
    rate.sleep();
  } // End of for(size_t i=0; i<yaw_step;++i)
  ROS_DEBUG("Finished looping over yaw_steps");
  
  // Commented because there will be no significant diff. between failure and successful  
  const double failure_w = 10.;
  const size_t n_failure = yaw_step - num_grasp_plan;
  res.process_cost = failure_w * (double) pow( (double)n_failure/yaw_step, 2 );
  
  const double failure_penalty = 100.;
  if(num_grasp_plan==0)
    res.process_cost += failure_penalty;
  
  ROS_DEBUG_STREAM("Grasp planning: DONE with " << num_grasp_plan << " grasp plan(s)");
  return true;
}
private:
ros::NodeHandle nh_;

ros::ServiceClient get_planning_scene_client_;

ros::Publisher state_validity_marker_array_pub_;
};

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "grasper");
  ros::NodeHandle nh;

//  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  GraspPlanner gp(nh);
  
  ros::ServiceServer plan_grasp_srv;
  plan_grasp_srv = nh.advertiseService("plan_grasp", &GraspPlanner::plan_grasp_srv_handle, &gp);
  
  ROS_INFO("Ready to plan grasps.");
  ros::spin();
  
  ros::shutdown();
  return 0;
};
