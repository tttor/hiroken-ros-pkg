#include <ros/ros.h>

#include "svm.h"
#include "libsvm_util.hpp"

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "libsvm_tester");
  ros::NodeHandle nh;

  std::string test_data_path;
//  test_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/libsvm-3.16/heart_scale";
  test_data_path = "/home/vektor/try_libsvm/libsvm-3.16/heart_scale";
  
  std::string fit_out_path;
  fit_out_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/fit.out";
  
  std::string svm_model_path;
  svm_model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/svm.model";
  
  FILE *input, *output;
  
	input = fopen(test_data_path.c_str(),"r");
	if(input == NULL)
	{
//		fprintf(stderr,"can't open input file %s\n",argv[i]);
		exit(1);
	}
	
	output = fopen(fit_out_path.c_str(),"w");
	if(output == NULL)
	{
//		fprintf(stderr,"can't open output file %s\n",argv[i+2]);
		exit(1);
	}
	
	SVMModel* model;
	if( (model=svm_load_model(svm_model_path.c_str())) == 0 )
	{
//		fprintf(stderr,"can't open model file %s\n",argv[i+1]);
		exit(1);
	}

  SVMNode* x;
  int max_nr_attr = 64;
  
  x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));

  int predict_probability=0;
  
  predict(model,predict_probability,max_nr_attr,x,input,output);
  
  svm_free_and_destroy_model(&model);
  free(x);
  free(line);
  fclose(input);
  fclose(output);
  
  return 0;
}
