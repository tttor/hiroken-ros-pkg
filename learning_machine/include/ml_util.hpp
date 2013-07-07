#ifndef ML_UTIL_HPP_INCLUDED
#define ML_UTIL_HPP_INCLUDED

#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include "engine.h"

namespace ml_util
{

// online-LWPR related params
static const size_t LWPR_INPUT_DIM = 76;// with planning horizon M= 5 on /home/vektor/rss-2013/data/ref/metadata.5.20130706.csv
static const size_t LWPR_OUTPUT_DIM = 1;

static const bool TUNED_LWPR_UPDATE_D = false;
static const double TUNED_LWPR_D = 0.9;
static const double TUNED_LWPR_ALPHA = 0.010;
static const double TUNED_LWPR_PEN = 0.010;

// Offline-SVR related params
static const size_t SVR_MAX_N_ATTR = 76 + 50;// plus tolerance= 50; with planning horizon M= 5 on /home/vektor/rss-2013/data/ref/metadata.5.20130706.csv

static const double TUNED_SVR_C = 3.000;
static const double TUNED_SVR_P = 0.010;
static const int TUNED_SVR_KERNEL_TYPE = 2;// RBF
static const double TUNED_SVR_GAMMA = 0.015;
static const std::string TUNED_SVR_DIM_RED = "pca";
static const size_t TUNED_SVR_LO_DIM = 50;

typedef 
enum 
{
  NO_ML=0, 
  NO_ML_BUT_COLLECTING_SAMPLES,
  SVR_OFFLINE,// in a batch mode, training is interleaved in between search 
  LWPR_ONLINE// in an online mode, the model is updated during search
} MLMode;

struct PrepData
{
  PrepData()
  { }

  //! The dimensionality reduction method
  std::string dim_red;
  
  //! The dimension of the reduced space
  size_t lo_dim;
    
  //! A transformation matrix from original space (hi_dim) to reduced space (lo_dim)
  Eigen::MatrixXd T;
  
  //! mean of input vectors, used to center the input vector
  Eigen::VectorXd mu_X;
  
  //! mean of output vectors
  Eigen::VectorXd mu_y;
};

std::vector<double>
get_y_fit(const std::string& fitting_data_path)
{
  std::vector<double> est_y;
  
  std::ifstream fitting_data(fitting_data_path.c_str());
  if(fitting_data.is_open())
  {
    while ( fitting_data.good() )
    {
      std::string line;

      std::getline(fitting_data,line);

      if(line.size() != 0)
        est_y.push_back( boost::lexical_cast<double>(line) );
    }
    fitting_data.close();
  }
  else
  {
   std::cout << "fitting_data.is_open(): Failed" << std::endl;
   return std::vector<double>();
  }
  
  return est_y;
}

std::vector<double>
get_y_true(const std::string& data_path)
{
  std::vector<double> y_true;
  
  std::ifstream sample_file(data_path.c_str());
  if(sample_file.is_open())
  {
    while ( sample_file.good() )
    {
      std::string line;

      std::getline(sample_file,line);
      
      // Parsing the csv, the output is at the last element
      std::vector<std::string> line_parts;
      boost::split( line_parts,line,boost::is_any_of(",") );
      
     if(line.size() != 0)
       y_true.push_back( boost::lexical_cast<double>(line_parts.back()) );
    }
    sample_file.close();
  }
  else
  {
   std::cout << "sample_file.is_open(): Failed" << std::endl;
   return std::vector<double>();
  }
  
  return y_true;
}

void
convert_var2mat(mxArray* var,Eigen::MatrixXd* mat)
{
  double* val_arr_ptr;
  val_arr_ptr = mxGetPr(var);

  double val_arr[mxGetNumberOfElements(var)];
  for(size_t i=0; i<mxGetNumberOfElements(var); ++i)
  {
    val_arr[i] = *(val_arr_ptr+i);
  }
  
  *mat = Eigen::Map<Eigen::MatrixXd>(val_arr,mxGetM(var),mxGetN(var));
}

void
convert_var2vec(mxArray* var,Eigen::VectorXd* vec)
{
  double* val_arr_ptr;
  val_arr_ptr = mxGetPr(var);

  double val_arr[mxGetNumberOfElements(var)];
  for(size_t i=0; i<mxGetNumberOfElements(var); ++i)
  {
    val_arr[i] = *(val_arr_ptr+i);
  }
  
  *vec = Eigen::Map<Eigen::VectorXd>(val_arr,mxGetNumberOfElements(var));
}

bool
preprocess_data(const std::string& in_data_path,const std::string& out_data_path,PrepData* prep_data)
{
  using namespace std;
  string cmd;// for any matlab commands
  
  // Fire up a matlab
  Engine *ep;
  if( !(ep = engOpen("")) ) 
  {
    cerr << "\nCan't start MATLAB engine\n";
    return false;
  }
  cout << "In matlab ;)" << endl;
  
  cmd = "format long;";
  engEvalString(ep,cmd.c_str());
  
  // Execute preprocess data cmds
  cmd = std::string("data = csvread('" + in_data_path + "');");
  engEvalString(ep,cmd.c_str());

  cmd = std::string("dim_red = '"+prep_data->dim_red+"';");
  engEvalString(ep,cmd.c_str());
  
  cmd = std::string("lo_dim = "+boost::lexical_cast<std::string>(prep_data->lo_dim)+";");
  engEvalString(ep,cmd.c_str());
  
  cmd = "addpath('~/Dropbox/01.Robotics/rss-2013/matlab-ws/util');";// where the preprocess_data() resides
  engEvalString(ep,cmd.c_str());
  
  cmd = "[data,prep_data] = preprocess_data(data,dim_red,lo_dim);";
  engEvalString(ep,cmd.c_str());
  
  // Write the new data
  cmd = std::string("csvwrite('" + out_data_path + "',data);");
  engEvalString(ep,cmd.c_str());
  
  // Retrieve preprocess data params for preprocessing the testing data
  cmd = "T = prep_data.T";
  engEvalString(ep,cmd.c_str());
  
  cmd = "mu_X = prep_data.mu_X";
  engEvalString(ep,cmd.c_str());
  
  cmd = "mu_y = prep_data.mu_y";
  engEvalString(ep,cmd.c_str());
  
  
  mxArray *var_T = NULL;
  mxArray *var_muX = NULL;
  mxArray *var_muy = NULL;
  if( ((var_T = engGetVariable(ep,"T")) == NULL) or
      ((var_muX = engGetVariable(ep,"mu_X")) == NULL) or
      ((var_muy = engGetVariable(ep,"mu_y")) == NULL)
    )
  {
    cout << "engGetVariable() == NULL" << endl;
    return false;
  }
  else 
  {
    convert_var2mat(var_T,&(prep_data->T));
    convert_var2vec(var_muX,&(prep_data->mu_X));
    convert_var2vec(var_muy,&(prep_data->mu_y));
  }
  
  // Turn of the matlab eng.
  cmd = "close";
  engEvalString(ep,cmd.c_str());
  engClose(ep);
  
  cout << "Out of matlab ;)" << endl;
  return true;
}

}// namespace ml_util
#endif // #ifndef ML_UTIL_HPP_INCLUDED
