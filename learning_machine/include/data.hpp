#ifndef DATA_HPP_INCLUDED
#define DATA_HPP_INCLUDED

#include "data.hpp"

typedef double Output;// which is the true geometric cost
typedef std::vector<double> Input;// which consists of feature-value, whose name is specified in the metadata
typedef std::map<std::string, double> RawInput;// which consists of features-name--feature-value pairs
typedef std::map<Input, Output> Data;

#endif // #ifndef DATA_HPP_INCLUDED
