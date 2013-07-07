#ifndef PLANNER_MANAGER_HPP_INCLUDED
#define PLANNER_MANAGER_HPP_INCLUDED

#include <ros/ros.h>
#include <ros/package.h>

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

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/config.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include <Eigen/Dense>

#include "grasp_planner/PlanGrasp.h"
#include "hiro_common/BenchmarkPath.h"
#include "planner_manager/Plan.h"
#include "planner_manager/Misc.h"

#include "tmm_utils.hpp"
#include "planner_manager.hpp"
#include "utils.hpp"

#include <lwpr.hh>
#include "svm.h"
#include "ml_util.hpp"
#include "libsvm_util.hpp"

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

bool
clear_n_ctamp_attempt_srv_handle(planner_manager::Misc::Request& req, planner_manager::Misc::Response& res);

private:
//! Plan manipulation plans.
/*!
  The output is optimal and symbollically-and-geometrically feasible.
  It is obvious that this planner manager does not interleave symbolic planning and geometric planning.
  Instead, it makes a call to the former first in order to obtain all symbollically feasible task plans, then validate each of them using geometric planning.
  This is the most straightforward way to do combined symbolic and geometric planning.
  It also employs the concept of task motion multigraph.
  
  size_t& ml_mode CANNOT be MLMode& ml_mode
  because it receive ml_mode from a service request, which does not have MLMode
*/
bool
plan(const size_t& ml_mode,const bool& rerun,const std::string& ml_hot_path,const std::string& log_path,std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol,std::vector<double>* ctamp_log);

bool
set_tidy_config();

double
get_cost2go(const TMMVertex& start,const TMMVertex& goal,const TaskMotionMultigraph& tmm);

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
std::map<std::string, arm_navigation_msgs::CollisionObject> movable_obj_messy_cfg_;
std::map<std::string, arm_navigation_msgs::CollisionObject> movable_obj_tidy_cfg_;
std::map<std::string, arm_navigation_msgs::CollisionObject> unmovable_obj_cfg_;

//! Keeps the number of samples that have been used to trained a SVR model
size_t n_data_;

size_t n_ctamp_attempt_;

//! For preprocessing related data
ml_util::PrepData prep_data_;

SVMModel* svr_model_;

LWPR_Object* lwpr_model_;

//! Number of ml model updates/trains
size_t n_ml_update_;

//! Only used in rerun mode, for inheriting motion planning from prev. run that used UCS and for computing true cost2go
TaskMotionMultigraph ucs_tmm_;
};// end of: class PlannerManager

#endif // #ifndef PLANNER_MANAGER_HPP_INCLUDED
