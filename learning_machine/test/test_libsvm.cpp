#include <ros/ros.h>

#include "svm.h"
#include "libsvm_util.hpp"

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "libsvm_tester");
  ros::NodeHandle nh;

  std::string tr_data_path;
  tr_data_path = "/home/vektor/rss-2013/data/ml_data/data.v4.libsvmdata";
  
  SVMParameter param;
  init_svmparam(&param);
  
  SVMNode* x_space;
  x_space = 0;
  
  SVMProblem prob;
   
  if( !read_problem(tr_data_path.c_str(),x_space,&prob,&param) )
  {
    ROS_ERROR("read_problem(): failed");
  }
  
  SVMModel* model;
  model = svm_train(&prob,&param);

  std::string model_path;
  model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/svm.v4.libsvmmodel";

  if( svm_save_model(model_path.c_str(),model) )
  {
    ROS_ERROR("Cannot save svm model.");
  }
  
  svm_free_and_destroy_model(&model);
  svm_destroy_param(&param);
  free(prob.y);
  free(prob.x);
  free(x_space);
  free(line);

  return 0;
}
