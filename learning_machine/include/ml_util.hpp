#ifndef ML_UTIL_HPP_INCLUDED
#define ML_UTIL_HPP_INCLUDED

typedef 
enum 
{
  NO_ML=0, 
  SVR_OFFLINE,// in a batch mode, training is interleaved in between search 
  LWPR_ONLINE// in an online mode, the model is updated during search
} MLMode;

#endif // #ifndef ML_UTIL_HPP_INCLUDED
