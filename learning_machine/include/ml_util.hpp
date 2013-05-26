#ifndef ML_UTIL_HPP_INCLUDED
#define ML_UTIL_HPP_INCLUDED

#include <boost/algorithm/string.hpp>
#include "engine.h"

namespace ml_util
{
static const bool TUNED_LWPR_UPDATE_D = true;
static const double TUNED_LWPR_D = 0.9;
static const double TUNED_LWPR_ALPHA = 1.5;
static const double TUNED_LWPR_PEN = 2.0;

static const double TUNED_SVR_C = 3.000;
static const double TUNED_SVR_P = 0.010;
static const int TUNED_SVR_KERNEL_TYPE = 2;// RBF
static const double TUNED_SVR_GAMMA = 0.015;

typedef 
enum 
{
  NO_ML=0, 
  NO_ML_BUT_COLLECTING_SAMPLES,
  SVR_OFFLINE,// in a batch mode, training is interleaved in between search 
  LWPR_ONLINE// in an online mode, the model is updated during search
} MLMode;

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

bool
pca(const std::string& in_data_path,const std::string& out_data_path)
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
  
  // Filter the data: input vector only
  cmd = std::string("data = csvread('" + in_data_path + "');");
  engEvalString(ep,cmd.c_str());

  cmd = "X = data(:,1:end-1);";
  engEvalString(ep,cmd.c_str());
  
  cmd = "y = data(:,end);";
  engEvalString(ep,cmd.c_str());
  
  // Do pca on X
  cmd = "[~,newX] = princomp(X);";
  engEvalString(ep,cmd.c_str());
  
  // Write the new data
  cmd = "new_data = [newX y]";
  engEvalString(ep,cmd.c_str());
  
  cmd = std::string("csvwrite('" + out_data_path + "',new_data);");
  engEvalString(ep,cmd.c_str());
  
  // Turn of the matlab eng.
  cmd = "close";
  engEvalString(ep,cmd.c_str());
  engClose(ep);
  
  cout << "Out of matlab ;)" << endl;
  return true;
}


}// namespace ml_util
#endif // #ifndef ML_UTIL_HPP_INCLUDED
