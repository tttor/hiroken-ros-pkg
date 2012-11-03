#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <vector>

#include "nn_machine/RunNet.h"
#include "nn_machine/TrainNet.h"

#include "floatfann.h"
#include "fann_cpp.h"

#include <ios>
#include <iostream>
#include <iomanip>
using std::cout;
using std::cerr;
using std::endl;
using std::setw;
using std::left;
using std::right;
using std::showpos;
using std::noshowpos;

using namespace std;
using namespace boost;

class NnMachine
{
public:
NnMachine(ros::NodeHandle& nh)
  :nh_(nh)
{
  training_data_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/for_training/training.data";
  csv_file_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/for_training/data.csv";
  metadata_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/for_training/metadata.csv";
  
  trained_net_path_ = "/home/vektor/hiroken-ros-pkg/nn_machine/net/trained.net";
  
  init();
}

~NnMachine()
{ }

bool
run_net_srv_handle(nn_machine::RunNet::Request& req, nn_machine::RunNet::Response& res)
{
  std::vector<double> feature_vals;

  for(std::vector<std::string>::const_iterator i=feature_ids_.begin(); i!=feature_ids_.end(); ++i)
  {
    bool found = false;
    for(std::vector<nn_machine::Feature>::const_iterator j=req.input.begin(); j!=req.input.end(); ++j)
    {
      if( !strcmp(i->c_str(), j->key.c_str()) )
      {
        feature_vals.push_back(j->val);
        found = true;
        break;
      }
    }
    if(!found)
      feature_vals.push_back(0.);
  }
  
  return run(feature_vals, &res);
}

bool
train_net_srv_handle(nn_machine::TrainNet::Request& req, nn_machine::TrainNet::Response& res)
{
//  if( train() )
//  {
//    res.msg = "Training: DONE";
//    return true;
//  }
//  else
//  {
//    res.msg = "Training: FAILED";
//    return false;
//  }
  cerr << "run_net: Get a call" << endl;
  res.msg = "in train";
  return true;
}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

FANN::neural_net net_;

std::string training_data_path_;
std::string csv_file_path_;
std::string metadata_path_;
std::string trained_net_path_;

std::vector<std::string> feature_ids_;

bool
init()
{
  const float learning_rate = 0.7f;
  
  const unsigned int num_layers = 3;

  const unsigned int num_input = 33;
  const unsigned int num_output = 1;

  const unsigned int num_hidden = 20;
  
  net_.create_standard(num_layers, num_input, num_hidden, num_output);

  net_.set_learning_rate(learning_rate);

  net_.set_activation_steepness_hidden(1.0);
  net_.set_activation_steepness_output(1.0);
  
  net_.set_activation_function_hidden(FANN::SIGMOID_SYMMETRIC_STEPWISE);
  net_.set_activation_function_output(FANN::SIGMOID_SYMMETRIC_STEPWISE);
  
  net_.print_parameters();
}

bool
train()
{
  const float desired_error = 0.1f;
  const unsigned int max_iterations = 100000;
  const unsigned int iterations_between_reports = 1000;  

  if ( !convert_csv_fann() )
    return false;
  
  cout << endl << "Training ..." << endl;

  FANN::training_data data;
  if ( data.read_train_from_file(training_data_path_.c_str()) )
  {
    net_.init_weights(data);

    net_.train_on_data(data, max_iterations, iterations_between_reports, desired_error);

    cout << endl << "Saving network." << endl;
    net_.save( trained_net_path_.c_str() );

    cout << endl << "Training: completed." << endl;
  }
}

bool
run(const std::vector<double>& feature_vals, nn_machine::RunNet::Response* res)
{
  //TODO check whether the trained_net file exists
  
  FANN::neural_net trained_net;
  trained_net.create_from_file( trained_net_path_.c_str() );

  fann_type in[feature_vals.size()];
  
  for(std::vector<double>::const_iterator i=feature_vals.begin(); i!=feature_vals.end(); ++i)
  {
    in[i-feature_vals.begin()] = *i;
  }

  fann_type *out;
  out = trained_net.run(in);

  res->output = out[0];
  
  return true;
}

bool
convert_csv_fann()
{
  // Get the number of samples==number of lines
  std::ifstream csv_file_in(csv_file_path_.c_str());
  
  size_t n = 0;
  if (csv_file_in.is_open())
  {
    while ( csv_file_in.good() )
    {
      std::string line;
      getline (csv_file_in,line);// Just for counting the line
      ++n;
    }
    --n;// Remove the bias from the method above!
    
    csv_file_in.close();
  }
  
  // Put the header information
  std::ofstream fann_data_file;
  fann_data_file.open(training_data_path_.c_str());// Overwrite
  
  // TODO make this adapt! This in only for 3 objects
  size_t n_input_feature = 33;
  size_t n_output = 1;
  
  std::ifstream metadata_in(metadata_path_.c_str());
  if (metadata_in.is_open())
  {
    std::string line;
    getline(metadata_in, line);// read only the first line
    
    boost::split( feature_ids_, line, boost::is_any_of(",") );
    
    if( !strcmp(feature_ids_.at(0).c_str(), std::string("").c_str())  )
    {
      feature_ids_.erase(feature_ids_.begin());// Note that eventhough there is no "," or the metadata is empty, the resulted vector still has 1 element which is an empty string.
      
      return false;
    }
    
    feature_ids_.erase(feature_ids_.end());// Erase the label "OUT"
    
    n_input_feature = feature_ids_.size();
  }
  
  fann_data_file << n << " " << n_input_feature << " " << n_output << endl;
  fann_data_file.close();
  
  // Read and Rewrite in the fann file. TODO it should use only one csv_file_path_
  std::ifstream csv_file_in_2(csv_file_path_.c_str());
  
  if (csv_file_in_2.is_open())
  {
    while ( csv_file_in_2.good() )
    {
      std::string line;
      getline (csv_file_in_2,line);
      
      std::vector<std::string> line_parts;
      boost::split( line_parts, line, boost::is_any_of(",") );
      
      // Write to the fann data file
      std::ofstream fann_data_file_2;
      fann_data_file_2.open(training_data_path_.c_str(), std::ios_base::app);
      
      // Write the input
      for(std::vector<std::string>::const_iterator i=line_parts.begin(); i!=line_parts.end()-1; ++i)
        fann_data_file_2 << *i << " ";
      fann_data_file_2 << endl;
      
      //Write the output
//      double out_val = boost::lexical_cast<float>(*(line_parts.end()-1)) ;// This leads to: Abnormal exception.
      std::string out_str = *(line_parts.end()-1);
      double out_val = atof ( out_str.c_str() );
      out_val /= 100.;
      
      fann_data_file_2 << out_val << endl;
      
      fann_data_file_2.close();
    }
    
    csv_file_in_2.close();
    return true;  
  }
  else
  {
    return false;
  }
}
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nn_machine");
  ros::NodeHandle nh;

  try
  {
    std::ios::sync_with_stdio(); // Syncronize cout and printf output
    
    NnMachine net(nh);
    
    ros::ServiceServer run_net_srv = nh.advertiseService("/run_net", &NnMachine::run_net_srv_handle, &net);
    ros::ServiceServer train_net_srv = nh.advertiseService("/train_net", &NnMachine::train_net_srv_handle, &net);    
    
    ROS_INFO("Spinning...");
    ros::spin();
  }
  catch (...)
  {
    cerr << endl << "Abnormal exception." << endl;
  }
  
  ros::shutdown();
  return 0;
}
