#ifndef ML_UTIL_HPP_INCLUDED
#define ML_UTIL_HPP_INCLUDED

#include <boost/algorithm/string.hpp>

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

}// namespace ml_util
#endif // #ifndef ML_UTIL_HPP_INCLUDED
