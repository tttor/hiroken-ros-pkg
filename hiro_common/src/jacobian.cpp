#include <kdl_parser/kdl_parser.hpp>

#include "jacobian.hpp"

Jacobian::Jacobian()
{
  initialized_ = false;
}

Jacobian::~Jacobian()
{
}

bool Jacobian::initialize(const std::string& start_link, const std::string& end_link)
{
  ROS_DEBUG("initialize jacobian from link %s to link %s", start_link.c_str(), end_link.c_str());
  start_link_name_ = start_link;
  end_link_name_ = end_link;

  ROS_VERIFY(urdf_.initParam("/robot_description"));
  ROS_VERIFY(kdl_parser::treeFromUrdfModel(urdf_, kdl_tree_));
  ROS_VERIFY(kdl_tree_.getChain(start_link, end_link, kdl_chain_));

  num_joints_ = kdl_chain_.getNrOfJoints();
  ROS_DEBUG("using %d joints", num_joints_);

  kdl_jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  kdl_jacobian_.resize(num_joints_);
  kdl_joint_positions_.resize(num_joints_);

  jacobian_ = Eigen::MatrixXd::Zero(6, num_joints_);

  initialized_ = true;
  return initialized_;
}

bool Jacobian::getJacobian(const Eigen::VectorXd& joint_values, Eigen::MatrixXd& jacobian)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(joint_values.size() == num_joints_);

  for(int i=0; i<num_joints_; i++)
  {
    kdl_joint_positions_.data[i] = joint_values[i];
  }

  kdl_jnt_to_jac_solver_->JntToJac(kdl_joint_positions_, kdl_jacobian_);
  jacobian = kdl_jacobian_.data;

  return true;
}

double Jacobian::getManipulabilityMeasure(const Eigen::VectorXd& joint_values)
{
  ROS_ASSERT(initialized_);

  Eigen::MatrixXd jac;

  getJacobian(joint_values, jac);

  Eigen::MatrixXd JJT = jac * jac.transpose();
  return sqrt(JJT.determinant());

}
