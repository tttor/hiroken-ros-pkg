// Adopted from Ludovic Righetti's code in usc-clmc-ros-pkg
// Also see: http://www.ros.org/wiki/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20Eigen

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
//#include <robot_info/robot_info.h>

#include <vector>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "assert.hpp"

class Jacobian
{
public:
  Jacobian();
  virtual ~Jacobian();

  bool initialize(const std::string& start_link,const std::string& end_link);

  bool getJacobian(const Eigen::VectorXd& joint_values, Eigen::MatrixXd& jacobian);

  double getManipulabilityMeasure(const Eigen::VectorXd& joint_values);

private:

  static const double ridge_factor_ = 10e-6;

  ros::NodeHandle node_handle_;

  bool initialized_;
  std::string start_link_name_;
  std::string end_link_name_;

  urdf::Model urdf_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray kdl_joint_positions_;
  KDL::Jacobian kdl_jacobian_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_jnt_to_jac_solver_;

  int num_joints_;

  Eigen::MatrixXd jacobian_;
};

#endif /* JACOBIAN_H_ */
