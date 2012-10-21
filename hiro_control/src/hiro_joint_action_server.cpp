#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

const double DEFAULT_GOAL_THRESHOLD = 0.01;

class HiroJointActionServer
{
private://These definition _must_ be here!
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
public:
  HiroJointActionServer(ros::NodeHandle &nh):
    nh_(nh),
    action_server_(nh_, "rarm_controller/follow_joint_trajectory",
                   boost::bind(&HiroJointActionServer::goalCB, this, _1),
                   boost::bind(&HiroJointActionServer::cancelCB, this, _1),
                   false),
    has_active_goal_(false)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");
    
    joint_names_.push_back("joint_chest_yaw");
    joint_names_.push_back("joint_rshoulder_yaw");
    joint_names_.push_back("joint_rshoulder_pitch");
    joint_names_.push_back("joint_relbow_pitch");
    joint_names_.push_back("joint_rwrist_yaw");
    joint_names_.push_back("joint_rwrist_pitch");
    joint_names_.push_back("joint_rwrist_roll");

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      double g, t;
      pn.param(ns + "/goal", g, DEFAULT_GOAL_THRESHOLD);
      pn.param(ns + "/trajectory", t, -1.0);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }
    pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);

    pub_controller_command_ = nh_.advertise<trajectory_msgs::JointTrajectory>("rarm_controller/command", 1);
    
    sub_controller_state_ = nh_.subscribe("rarm_controller/feedback_states", 1, &HiroJointActionServer::controllerStateCB, this);
    
    action_server_.start();
  }

  ~HiroJointActionServer()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

private:
  void goalCB(GoalHandle gh)
  {
    // Ensures that the joints in the goal match the joints we are commanding.
    ROS_DEBUG("Received goal: goalCB");
    ROS_INFO("Received goal: goalCB");
    if (!is_joint_set_ok(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      ROS_DEBUG("Received new goal, canceling current goal");
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Sends the trajectory along to the controller
    ROS_INFO("Publishing trajectory; sending commands to the joint controller ");
    current_traj_ = active_goal_.getGoal()->trajectory;
    pub_controller_command_.publish(current_traj_);
  }

  void cancelCB(GoalHandle gh)
  {
    ROS_DEBUG("Received action cancel request");
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }
 
  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
  {
    //ROS_DEBUG("Checking controller state feedback");
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
    {
      //ROS_DEBUG("No active goal, ignoring feedback");
      return;
    }
    if (current_traj_.points.empty())
    {
      ROS_DEBUG("Current trajecotry is empty, ignoring feedback");
      return;
    }
    /* NOT CONCERNED ABOUT TRAJECTORY TIMING AT THIS POINT
    if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
      return;
    */
    if (!is_joint_set_ok(joint_names_, msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }
    
    // Checking for goal constraints
    // Checks that we have ended inside the goal constraints
    ROS_DEBUG("Checking goal contraints");
    bool inside_goal_constraints = true;
    int last = current_traj_.points.size() - 1;
    for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints; ++i)
    {
      double abs_error = fabs(msg->actual.positions[i]-current_traj_.points[last].positions[i]);
      double goal_constraint = goal_constraints_[msg->joint_names[i]];
      if (goal_constraint >= 0 && abs_error > goal_constraint)
      {
        inside_goal_constraints = false;
      }
      ROS_DEBUG("Checking constraint: %f, abs_errs: %f", goal_constraint, abs_error);
    }
    
    if (inside_goal_constraints)
    {
      ROS_INFO("Inside goal contraints, return success for action");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
  
  // Helper functions.
  static bool is_joint_set_ok(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }
  
  ros::NodeHandle nh_;
  JTAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  control_msgs::FollowJointTrajectoryFeedbackConstPtr last_controller_state_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hiro_joint_action_server");
  ros::NodeHandle nh;//("~");
  HiroJointActionServer hjas(nh);
  
  ROS_INFO("hiro_joint_action_server:: UP and RUNNING");
  ros::spin();

  return 0;
}
