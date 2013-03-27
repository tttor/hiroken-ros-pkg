#ifndef PLANNER_MANAGER_HPP_INCLUDED
#define PLANNER_MANAGER_HPP_INCLUDED

#include <ros/ros.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

#include <fstream>
#include <algorithm>
#include <functional>
#include <limits>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/algorithm/string.hpp>
#include <boost/config.hpp>
#include <boost/random.hpp>

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include "grasp_planner/PlanGrasp.h"
#include "hiro_common/BenchmarkPath.h"
#include "planner_manager/Plan.h"

#include "tmm_utils.hpp"
#include "planner_manager.hpp"
#include "obj_cfg_rw.hpp"
#include "utils.hpp"

#include <lwpr.hh>
#include "libsvm_util.hpp"

typedef enum 
{
  NO_ML=1, 
  SVR_OFFLINE=2, 
  LWPR_OFFLINE=3,
  LWPR_ONLINE=4
} MLMode;

class PlannerManager
{
friend class SymbolicPlannerManager;
friend class GeometricPlannerManager;

public:
//! A constructor.
/*!
  ...
*/
PlannerManager(ros::NodeHandle& nh);
//! A destructor.
/*!
  The destructor does nothing.
*/
~PlannerManager();

void 
collision_object_cb(const arm_navigation_msgs::CollisionObject::ConstPtr& msg);

bool
plan_srv_handle(planner_manager::Plan::Request& req, planner_manager::Plan::Response& res);

private:
//! Plan manipulation plans.
/*!
  The output is optimal and symbollically-and-geometrically feasible.
  It is obvious that this planner manager does not interleave symbolic planning and geometric planning.
  Instead, it makes a call to the former first in order to obtain all symbollically feasible task plans, then validate each of them using geometric planning.
  This is the most straightforward way to do combined symbolic and geometric planning.
  It also employs the concept of task motion multigraph.
*/
bool
plan(const size_t& mode,std::vector<trajectory_msgs::JointTrajectory>* man_plan);

bool
set_tidy_config();

//! A task motion multigraph variable
/*!
  Task plan space is encoded here.
*/
TaskMotionMultigraph tmm_;
TMMVertex tmm_root_;
TMMVertex tmm_goal_;
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;
//! A helper variable
/*!
  More ...
*/
std::string planner_manager_path_;
//! A helper variable
/*!
  More ...
*/
std::map<std::string, arm_navigation_msgs::CollisionObject> messy_cfg_;
std::map<std::string, arm_navigation_msgs::CollisionObject> tidy_cfg_;
};// end of: class PlannerManager

#endif // #ifndef PLANNER_MANAGER_HPP_INCLUDED
