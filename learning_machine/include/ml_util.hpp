#ifndef ML_UTIL_HPP_INCLUDED
#define ML_UTIL_HPP_INCLUDED

#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include "engine.h"
#include "data.hpp"

namespace ml_util
{

// online-LWPR related params
static const size_t LWPR_INPUT_DIM = 76;// with planning horizon M= 5 on /home/vektor/rss-2013/data/ref/metadata.5.20130706.csv
static const size_t LWPR_OUTPUT_DIM = 1;
//===July 8, 2013, 20:10===================================================================================================
//10-fold CV on LWPR with n_data=  10000, d= 76 
//Lowest MSE= 1.14766 with update_D= 0.000 init_D_mul= 0.100 init_alpha_mul= 0.010 pen= 0.010
//% ===Aug 26, 2013, 22:19===================================================================================================
//% 10-fold CV on LWPR with n_data=  5000, d= 76 
//% Lowest MSE= 0.77394 with update_D= 0.000 init_D_mul= 0.100 init_alpha_mul= 0.010 pen= 0.010
static const bool TUNED_LWPR_UPDATE_D = false;
static const double TUNED_LWPR_D = 0.100;
static const double TUNED_LWPR_ALPHA = 0.010;
static const double TUNED_LWPR_PEN = 0.010;

// Offline-SVR related params
static const size_t SVR_MAX_N_ATTR = 76 + 50;// plus tolerance= 50; with planning horizon M= 5 on /home/vektor/rss-2013/data/ref/metadata.5.20130706.csv
//===July 8, 2013, 18:41===================================================================================================
//CV on kernel= RBF and on SVR= e-SVR with n_data= 9262 n_fea= 50 dim_red= pca low_dim= 50 
//Lowest MSE= 0.14763 with c= 32.000 epsilon= 0.010 gamma= 0.002
//===Aug 27, 2013, 16:11===================================================================================================
//CV on kernel= RBF and on SVR= e-SVR with n_data= 10000 n_fea= 50 
//Lowest MSE= 0.50959 with c= 8.000 epsilon= 0.010 gamma= 0.002 

static const double TUNED_SVR_C = 8.000;
static const double TUNED_SVR_P = 0.010;
static const int TUNED_SVR_KERNEL_TYPE = 2;// RBF
static const double TUNED_SVR_GAMMA = 0.002;
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

double
get_mean(const std::vector<double>& data)
{
  double total = 0.;
  
  for(size_t i=0; i<data.size(); ++i)
  {
    total += data.at(i);
  }
  
  return (total/data.size());
}

double
get_var(const std::vector<double>& data)
{
  double mu;
  mu = get_mean(data);
  
  double sum = 0.;
  for(size_t i=0; i<data.size(); ++i)
  {
    sum += pow(data.at(i)-mu,2);
  }
  
  if(data.size() > 1)
    return (sum/(data.size()-1));
  else
    return (sum/data.size());
}

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

std::vector<Input>
get_X(const std::string& data_path)
{
  std::vector<Input> X;
  
  std::ifstream sample_file(data_path.c_str());
  if(sample_file.is_open())
  {
    while ( sample_file.good() )
    {
      std::string line;
      std::getline(sample_file,line);
      
      if(line.size() != 0)
      {
        // Parsing the csv, the input feature vector values are from all but the last element
        std::vector<std::string> line_parts;
        boost::split( line_parts,line,boost::is_any_of(",") );
      
        Input x;
        for(size_t i=0; i<line_parts.size()-1; ++i)// excluding the last part: the output
          x.push_back( boost::lexical_cast<double>(line_parts.at(i)) );
        
        X.push_back(x);
      }
    }
    sample_file.close();
  }
  else
  {
    std::cout << "sample_file.is_open(): Failed on " << data_path << std::endl;
  }
  
  return X;
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
      
      if(line.size() != 0)
      {
        // Parsing the csv, the output is at the last element
        std::vector<std::string> line_parts;
        boost::split( line_parts,line,boost::is_any_of(",") );
        
        y_true.push_back( boost::lexical_cast<double>(line_parts.back()) );
      }
      
    }
    sample_file.close();
  }
  else
  {
   std::cout << "sample_file.is_open(): Failed on " << data_path << std::endl;
   return std::vector<double>();
  }
  
  return y_true;
}

std::vector<double>
get_y_true_libsvmdata(const std::string& data_path)
{
  std::vector<double> y_true;
  
  std::ifstream sample_file(data_path.c_str());
  if(sample_file.is_open())
  {
    while ( sample_file.good() )
    {
      std::string line;
      std::getline(sample_file,line);
      
      if(line.size() != 0)
      {
        // Parsing the libsvmdata, the output is at the first part
        std::vector<std::string> line_parts;
        boost::split( line_parts,line,boost::is_any_of(" ") );// yes, delimited by a space

        y_true.push_back( boost::lexical_cast<double>(line_parts.front()) );
      }
    }
    sample_file.close();
  }
  else
  {
   std::cout << "sample_file.is_open(): Failed on " << data_path << std::endl;
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
preprocess(const std::vector<Input>& X,const PrepData& prep_data, std::vector<Input>* preprocessed_X)
{
  for(size_t i=0; i < X.size(); ++i)
  {
    Input x;
    x = X.at(i);
    
    // All below should mimic the proprocess_data() routine implemented in matlab
    double* x_ptr = &x[0];
    Eigen::Map<Eigen::VectorXd> tmp_x(x_ptr,x.size());
    
    // Centering; PCA needs centered_x
    Eigen::VectorXd centered_x;
    centered_x = tmp_x - prep_data.mu_X;
    
    // Project to new space
    Eigen::VectorXd new_x;
    new_x = centered_x.transpose() * prep_data.T;
    
    // Reduce dim
    Eigen::VectorXd lodim_x;
    lodim_x = new_x.head(prep_data.lo_dim);
    
    // Convert back to std::vector
    x.clear();
    x.resize(lodim_x.size());
    for(size_t i=0; i<lodim_x.size(); ++i)
      x.at(i) = lodim_x(i);
      
    preprocessed_X->push_back(x);
  }
  
  return true;
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
