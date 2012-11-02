#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <vector>

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
  training_data_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/hot/training.data";
  csv_file_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/hot/data.csv";
  metadata_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/hot/metadata.csv";
  
  init();
}

~NnMachine()
{ }

bool
train()
{
  const float desired_error = 0.1f;
  const unsigned int max_iterations = 300000;
  const unsigned int iterations_between_reports = 1000;  

  if ( !convert_csv_fann() )
    return false;
  
  cout << endl << "Training ..." << endl;

  FANN::training_data data;
  if (data.read_train_from_file(training_data_path_.c_str()))
  {
    // Initialize and train the network with the data
    net_.init_weights(data);

//    cout << "Max Epochs " << setw(8) << max_iterations << ". "
//        << "Desired Error: " << left << desired_error << right << endl;
//      net_.set_callback(print_callback, NULL);

    net_.train_on_data(data, max_iterations, iterations_between_reports, desired_error);

//    cout << endl << "Testing network." << endl;

//    for (unsigned int i = 0; i < data.length_train_data(); ++i)
//    {
//        // Run the network on the test data
//        fann_type *calc_out = net_.run(data.get_input()[i]);

//        cout << "XOR test (" << showpos << data.get_input()[i][0] << ", " 
//             << data.get_input()[i][1] << ") -> " << *calc_out
//             << ", should be " << data.get_output()[i][0] << ", "
//             << "difference = " << noshowpos
//             << fann_abs(*calc_out - data.get_output()[i][0]) << endl;
//    }
    
    cout << endl << "Saving network." << endl;

    // Save the network in floating point and fixed point
    net_.save("/home/vektor/hiroken-ros-pkg/nn_machine/net/trained.net");

    cout << endl << "Training: completed." << endl;
  }
}

bool
run()
{
//  fann_type *calc_out;
//  fann_type input[2];

//  FANN::neural_net trained_net;
//  trained_net.create_from_file("/home/vektor/hiroken-ros-pkg/nn_machine/net/trained.net");

//  input[0] = -1;
//  input[1] = 1;
//  calc_out = net.run(input);

//  printf("xor test (%f,%f) -> %f\n", input[0], input[1], calc_out[0]);
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
    
    net.train();
    
    ROS_INFO("Spinning...");
    ros::spin();
  }
  catch (...)
  {
      cerr << endl << "Abnormal exception." << endl;
  }
  
  return 0;
}
