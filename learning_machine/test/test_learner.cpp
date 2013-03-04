#include <ros/ros.h>

#include "learning_machine/Train.h"

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "ml_tester");
  ros::NodeHandle nh;

  std::vector<std::string> tmm_paths;
  
  XmlRpc::XmlRpcValue v;
  nh.param("/tmm_paths", v, v);
  for(int i =0; i < v.size(); i++)
    tmm_paths.push_back(v[i]);
  
  ros::service::waitForService("/train");
  ros::ServiceClient client;
  client = nh.serviceClient<learning_machine::Train>("/train");
    
  learning_machine::Train::Request req;
  learning_machine::Train::Response res;
  
  req.tmm_paths = tmm_paths;  
 
  ROS_INFO("calling /train srv ...");
  if( !client.call(req,res) )
  {
    return 1;
  }
  
  return 0;
}
/*
Cause Service call failed: service [/train] responded with an error: bad lexical cast: source type value could not be interpreted as target
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/run.3obj.20130223.b.0/tmm.dot");
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/datarun.3obj.20130220.0/tmm.dot");
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/datarun.3obj.20130220.2/tmm.dot");
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/datarun.3obj.20130220.3/tmm.dot");
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/datarun.3obj.20130220.4/tmm.dot");
//  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/run.3obj.20130223.4/tmm.dot");
  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/run.3obj.20130225.b.2/tmm.dot");
  req.tmm_paths.push_back("/home/vektor/rss-2013/data/baseline/v.2/3obj/run.3obj.20130225.b.3/tmm.dot");
*/
