#ifndef DATA_HPP_INCLUDED
#define DATA_HPP_INCLUDED

#include "data.hpp"
#include "utils.hpp"

#include <boost/algorithm/string.hpp>
#include <fstream>

typedef double Output;// which is the true geometric cost
typedef std::vector<double> Input;// which contains feature-values, whose names are specified in the metadata
typedef std::map<std::string, double> RawInput;// which consists of features-name--feature-value pairs, for sparse data
typedef std::map<Input, Output> Data;// inforce unique data only, in particular unique key(=Input)
typedef std::set<Input> InputOnlyData;// inforce unique data only, in particular unique key(=Input)

namespace data_util
{
//! This gets the upper bound of input-feature size
/*!
  feature name = feature label
*/
std::vector<std::string>
get_labels(const std::string& metadata_path)
{
  std::vector<std::string> feature_names;
  
  // Read from a csv file containing metadata
  std::ifstream metadata_in(metadata_path.c_str());
  
  if ( metadata_in.is_open() )
  {
    std::string metadata;
    
    getline(metadata_in, metadata);// Read only the first line; the only line here
    metadata_in.close();
    
    // Parse the metadata, put into a vector
    boost::split( feature_names, metadata, boost::is_any_of(",") );

    // Note that eventhough there is no "," or the metadata is empty, the resulted vector still has 1 element which is an empty string.
    if( feature_names.at(0).size()==0 )
    {
      std::cerr << "metadata file is corrupt." << std::endl;
      feature_names.clear();
    }
    else
    {
      // Remove the OUT label
      std::vector<std::string>::iterator OUT_it;
      OUT_it = std::find(feature_names.begin(), feature_names.end(), "OUT");
      if(OUT_it!=feature_names.end())
        feature_names.erase(OUT_it);
    }
  }
  else 
  {
    std::cerr << "Unable to open:" << metadata_path << std::endl;
  }
  
  return feature_names;
}

//std::vector<std::string>
//get_labels()
//{
//  return get_labels("/home/vektor/rss-2013/data/ml_data/metadata.csv");
//}

bool
convert(RawInput& r_in,Input* in,const std::vector<std::string>& labels)
{
  if(labels.empty())
  {
    std::cerr << "Cannot convert raw inputs: labels.empty()" << std::endl;
    return false;
  }
  
  for(std::vector<std::string>::const_iterator i=labels.begin(); i!=labels.end(); ++i)
  {
    RawInput::iterator j;
    j = r_in.find(*i);
    
    if( j!=r_in.end() )
      in->push_back( j->second );
    else
      in->push_back(0.);// The default val e.g. Make it a point and at the Origin for object's pose; Set not-exist value for symbolic features
  }
  
  return true;
}

bool
write_libsvm_data(const Data& data,const std::string& data_out_path,std::ios_base::openmode mode = std::ios_base::out)
{
  std::ofstream data_out;
  data_out.open(data_out_path.c_str(),mode);

//  std::cerr << "data.size()= " << data.size() << std::endl;
  for(Data::const_iterator i=data.begin(); i!=data.end(); ++i)
  {
    data_out << i->second << " ";
    
    for(Input::const_iterator j=i->first.begin(); j!=i->first.end(); ++j)
    {
      size_t idx;
      idx = j - i->first.begin() + 1;// libsvm manual: <index> is an integer starting from 1
      
      data_out << idx << ":" << *j << " ";
    }
    
    data_out << std::endl;
  }
  
  data_out.close();
  return true; 
}

//! For testing/predicting purpose where the output y is unknown
bool
write_libsvm_data(const Input& in,const std::string& data_out_path,std::ios_base::openmode mode = std::ios_base::out)
{
  Data data;
  data[in] = 0.;// arbitrary_y;

  return write_libsvm_data(data,data_out_path,mode);
}

//! Write to a csv file
/*!
  Accept only datatype= Data 
*/
void
write_csv_data(const Data& data,const std::string& csv_path,std::ios_base::openmode mode = std::ios_base::out)
{
  std::ofstream csv;
  csv.open( csv_path.c_str(),mode );

  for(Data::const_iterator i=data.begin(); i!=data.end(); ++i)
  {
    for(Input::const_iterator j=i->first.begin(); j!=i->first.end(); ++j)
    {
      csv << *j << ",";// Write input=feature values
    }
    csv << i->second << std::endl;
  }
  csv.close();
}

//! Note that this conversion also remove any duplicates
bool
convert_csv2libsvmdata(const std::string& csv_path,const std::string& libsvmdata_path)
{
  using namespace std; 
  cerr << "convert_csv2libsvmdata : BEGIN" << endl;
  
  Data data;
  size_t n_line = utils::get_n_lines(csv_path);
//  cerr << "n_line= " << n_line << endl;
  
  ifstream csv(csv_path.c_str());
  if (csv.is_open())
  {
    size_t n_duplicate = 0;
    for(size_t i=0; i<n_line; ++i)
    {
      string line;
      getline (csv,line);
      cerr << "line= " << line << endl;
      
      // Parse
      vector<string> str_vals;
      boost::split( str_vals,line, boost::is_any_of(",") );
      if(str_vals.at(0).size() == 0)
      {
        cerr << "str_vals.at(0).size() == 0 [csv is CORRUPTED]" << endl;
        return false;
      }
      
      // Obtain the input vector and the output
      Input in;
      for(vector<string>::const_iterator i=str_vals.begin(); i!=str_vals.end()-1; ++i)// minus the single output value
        in.push_back( boost::lexical_cast<double>(*i) );
        
      Output out;
      out = boost::lexical_cast<double>(str_vals.back());
      
      // Wrap up
      bool unique;
      unique = data.insert( make_pair(in,out) ).second;
      
      if(!unique) ++n_duplicate;
    }
//    cerr << "n_duplicate= " << n_duplicate << endl;
    csv.close();
  }
  else
  {
    cerr << "Cannot open: " << csv_path << endl;
    return false;
  }

  cerr << "convert_csv2libsvmdata : END (with write_libsvm_data(data,libsvmdata_path))" << endl;   
  return write_libsvm_data(data,libsvmdata_path);
}

}// namespace data_utils

#endif // #ifndef DATA_HPP_INCLUDED
