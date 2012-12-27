#include <ros/ros.h>

#include "sensor_manager/Sense.h"
#include "hiro_sensor/See.h"

class SensorManager
{
public:
SensorManager(ros::NodeHandle nh)
:nh_(nh)
{ }

bool
sense_srv_handle(sensor_manager::Sense::Request& root_req, sensor_manager::Sense::Response& root_res)
{
  switch(root_req.id)
  {
    case 1:// Vision, see!
    {
      ros::service::waitForService("/see");
    
      ros::ServiceClient see_client;
      see_client = nh_.serviceClient<hiro_sensor::See> ("/see");
      
      hiro_sensor::See::Request req;
      hiro_sensor::See::Response res;

      req.random = root_req.uint_args.at(0);// Whether using a random messy cfg or a hardcoded testbed      
      req.n = root_req.uint_args.at(1);// number of expected sensed objects
      req.path = root_req.string_args.at(0);// path for replay messy_cfg
        
      if( !see_client.call(req,res) ) 
      {
        ROS_WARN("A call to /see srv: FAILED");
        return false;
      }
      break;
    }
    default:
      ROS_WARN("Unknown sensing id, sense nothing!");
  }
  return true;
}

private:
//! A ROS node handler
/*!
  Useless if outside ROS.
*/
ros::NodeHandle nh_;
};

int 
main(int argc, char **argv)
{ 
  ros::init(argc, argv, "sensor_mgr");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  SensorManager sensor(nh);

  ros::ServiceServer see_srv;
  see_srv = nh.advertiseService("/sense", &SensorManager::sense_srv_handle, &sensor);
  
  ROS_INFO("sensor_mgr: spinning...");
  ros::spin();
  
  return 0;
}
