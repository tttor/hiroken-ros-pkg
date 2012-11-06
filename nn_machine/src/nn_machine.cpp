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
  training_data_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/trn/training.data";
  raw_data_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/trn/data.csv";
  metadata_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/trn/metadata.csv";
  prep_data_path_ = "/home/vektor/hiroken-ros-pkg/planner_manager/data/trn/prep_data.csv";
  
  trained_net_path_ = "/home/vektor/hiroken-ros-pkg/nn_machine/net/trained.net";
  
  std::ifstream metadata_in(metadata_path_.c_str());
  if (metadata_in.is_open())
  {
    std::string line;
    getline(metadata_in, line);// read only the first line
    
    boost::split( feature_ids_, line, boost::is_any_of(",") );
    
    if( !strcmp(feature_ids_.at(0).c_str(), std::string("").c_str())  )
    {
      feature_ids_.erase(feature_ids_.begin());// Note that eventhough there is no "," or the metadata is empty, the resulted vector still has 1 element which is an empty string.
      ROS_ERROR("metadata file is empty");
    }
    
    feature_ids_.erase(feature_ids_.end());// Erase the label "OUT"
  }
  
  init();
  fill_h_lookup();
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
  
//  // Write, only if this is new vertex
//  pair< set<size_t>::iterator,bool > inserted;
//  inserted = vertices_.insert(req.vertex);
//  
//  if(inserted.second)
//  {
//    std::ofstream test_data_out;
//    test_data_out.open("/home/vektor/hiroken-ros-pkg/planner_manager/data/test/tb.3.csv", std::ios_base::app);
//    for(std::vector<double>::const_iterator i=feature_vals.begin(); i!=feature_vals.end()-1; ++i)
//      test_data_out << *i << ",";
//    test_data_out << *(feature_vals.end()-1);
//    test_data_out << "," << req.vertex;
//    test_data_out << endl;
//    test_data_out.close();
//  }
//  res.output = 0.;
//  return true;
  
  return run(req.vertex, &res);
//  return run(feature_vals, &res);
}

bool
fill_h_lookup()
{
  // Read the lookup table file, put into a map
//  std::ifstream h_lookup_in("/home/vektor/hiroken-ros-pkg/nn_machine/net/h_lookup_plsregress.tb2.csv");
//  std::ifstream h_lookup_in("/home/vektor/hiroken-ros-pkg/nn_machine/net/h_lookup_plsregress.tb3.csv");  
  
  std::ifstream h_lookup_in("/home/vektor/hiroken-ros-pkg/nn_machine/net/h_lookup_nn.tb1.csv");
  
  if ( h_lookup_in.is_open() )
  {
    while ( h_lookup_in.good() )
    {
      std::string line;
      getline(h_lookup_in, line);
      
      std::vector<std::string> line_parts;
      boost::split( line_parts, line, boost::is_any_of(",") );
      
      if(line_parts.size() > 1)// If not the empty last row
        h_lookup_.insert(  pair<size_t, double>(atoi(line_parts.at(0).c_str()), atof(line_parts.at(1).c_str()))  );
    }
    
    h_lookup_in.close();
  }
  else
  {
    ROS_ERROR("Can not open the lookup h(n)");
    return false;
  }
  
  return true;
}

bool
run(const size_t& vertex, nn_machine::RunNet::Response* res)
{
  double h;
  
  h = h_lookup_[vertex];
  post_process_h(&h);
  
  res->output = h;
  
  return true;
}

bool
post_process_h(double* h)
{
  (*h) *= 100.;
  
  return true;
}

bool
train_net_srv_handle(nn_machine::TrainNet::Request& req, nn_machine::TrainNet::Response& res)
{
  if( train() )
  {
    res.msg = "Training: DONE";
    return true;
  }
  else
  {
    res.msg = "Training: FAILED";
    return false;
  }
}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;

FANN::neural_net net_;

std::string training_data_path_;
std::string raw_data_path_;
std::string metadata_path_;
std::string trained_net_path_;
std::string prep_data_path_;

std::vector<std::string> feature_ids_;

std::set<size_t> vertices_;
std::map<size_t, double> h_lookup_;

//std::vector<double> mu_;
//std::vector<double> std_;

//size_t n_;

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
  
  return true;
}

bool
train()
{
  const float desired_error = 0.10f;
  const unsigned int max_iterations = 100000;
  const unsigned int iterations_between_reports = 10000;  

  if( !preprocess_data() )
    return false;

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
  
  return true;
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
preprocess_data()
{
	// read raw data, put in std::vector< std::vector<double> > in_data and std::vector<double>
	std::vector< std::vector<double> > in_data;
	std::vector<double> out_data;

  std::ifstream raw_data_in(raw_data_path_.c_str());
  
  if ( raw_data_in.is_open() )
  {
    while ( raw_data_in.good() )
    {
      std::string line;
      getline (raw_data_in,line);
      
      std::vector<std::string> line_parts;
      boost::split( line_parts, line, boost::is_any_of(",") );
      
      // Write the input into in_data
      std::vector<double> in_datum;
      for(std::vector<std::string>::const_iterator i=line_parts.begin(); i!=line_parts.end()-1; ++i)
      {
        std::string val_str = *i;
        double val = atof( val_str.c_str() );
        
        in_datum.push_back(val);
      }
      in_data.push_back(in_datum);
      
      std::string out_str = *(line_parts.end()-1);
      out_data.push_back( atof( out_str.c_str() ) );
    }
    in_data.erase(in_data.end()-1);
    out_data.erase(out_data.end()-1);
    
    raw_data_in.close();
  }
  
	// Preprocess
	if( !preprocess_in_data(&in_data) or !preprocess_out_data(&out_data) )
	  return false;
	
	// Write preprocessed data in prep_data.csv
	std::ofstream prep_data_out;
  prep_data_out.open(prep_data_path_.c_str());
  prep_data_out << "";
  prep_data_out.close();
  
  prep_data_out.open(prep_data_path_.c_str(), std::ios_base::app);
  for(std::vector< std::vector<double> >::const_iterator i=in_data.begin(); i!=in_data.end(); ++i)
  {
    for(std::vector<double>::const_iterator j=i->begin(); j!=i->end(); ++j)
    {
      prep_data_out << *j << ",";
    }
    prep_data_out << out_data.at(i-in_data.begin()) << endl;
  }
  prep_data_out.close();

	return true;
}

bool 
preprocess_in_data(const std::vector<double>& mus, const std::vector<double>& sigmas, std::vector< std::vector<double> >* in_data)
{
	// normalize the in_data with mus_
	for(std::vector< std::vector<double> >::iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::iterator j=i->begin(); j!=i->end(); ++j)
		{
			(*j) -= mus.at( j-(i->begin()) );
		}
	}
	
		// Normalize the in_data with std_
	for(std::vector< std::vector<double> >::iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::iterator j=i->begin(); j!=i->end(); ++j)
		{
			(*j) /= sigmas.at(j-(i->begin()));
		}
	}
	
	return true;
}

bool
preprocess_in_data(std::vector< std::vector<double> >* in_data)
{
	size_t n = in_data->size();
	size_t n_feature = in_data->at(0).size();
	
	// calc the mean mus for all feature
	std::vector<double> mus(n_feature, 0.);
	
	for(std::vector< std::vector<double> >::const_iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::const_iterator j=i->begin(); j!=i->end(); ++j)
		{
			mus.at( j-(i->begin()) ) += (*j);// Sum up to get the total
		}
	}

	for(std::vector<double>::iterator i=mus.begin(); i!=mus.end(); ++i)
	{
		(*i) /= n;
	}

	// normalize the in_data with mus_
	for(std::vector< std::vector<double> >::iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::iterator j=i->begin(); j!=i->end(); ++j)
		{
			(*j) -= mus.at( j-(i->begin()) );
		}
	}

	// calc std for each feature. Note that up to this point, mu of each feature is already zero (normalized)
	std::vector<double> sigmas(n_feature, 0.);
	for(std::vector< std::vector<double> >::const_iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::const_iterator j=i->begin(); j!=i->end(); ++j)
		{
			sigmas.at( j-(i->begin()) ) += pow(*j, 2);
		}
	}

	for(std::vector<double>::iterator i=sigmas.begin(); i!=sigmas.end(); ++i)
	{
		(*i) /= (n-1);
		(*i) = sqrt(*i);
	}

	// Normalize the in_data with std_
	for(std::vector< std::vector<double> >::iterator i=in_data->begin(); i!=in_data->end(); ++i)
	{
		for(std::vector<double>::iterator j=i->begin(); j!=i->end(); ++j)
		{
			(*j) /= sigmas.at(j-(i->begin()));
		}
	}
	
//	cout << "MUs= " << endl;
//	for(std::vector<double>::const_iterator i=mus.begin(); i!=mus.end(); ++i)
//	  cout << *i << endl;

//	cout << "SIGMAs= " << endl;
//	for(std::vector<double>::const_iterator i=sigmas.begin(); i!=sigmas.end(); ++i)
//	  cout << *i << endl;
	  	  
	return true;
}

bool
preprocess_out_data(std::vector<double>* out_data)
{
	// Preprocess the output
	for(std::vector<double>::iterator i=out_data->begin(); i!=out_data->end(); ++i)
	{
		const double scale = 0.01;
		
		(*i) *= scale;
	}
	
	return true;
}

bool
convert_csv_fann()
{
  // Get the number of samples==number of lines
  std::ifstream csv_file_in(prep_data_path_.c_str());
  
  size_t n = 0;
  if (csv_file_in.is_open())
  {
    while ( csv_file_in.good() )
    {
      std::string line;
      getline(csv_file_in,line);// Just for counting the line
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
  
  fann_data_file << n << " " << n_input_feature << " " << n_output << endl;
  fann_data_file.close();
  
  // Read and Rewrite in the fann file.
  std::ifstream csv_file_in_2(prep_data_path_.c_str());
  
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
      {
        fann_data_file_2 << *i << " ";
      }
      fann_data_file_2 << endl;
      
      //Write the output
      fann_data_file_2 << *(line_parts.end()-1) << endl;
      
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

