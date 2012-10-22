#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "hiro_sensor/Sense.h"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

// TODO wrap this data in a header
// Based on observation in Jul 1, 2012
static const double TABLE_HEIGHT = 0.835;
static const double TABLE_RADIUS = 0.600;
static const double TABLE_THICKNESS = 0.050;
static const double DIST_TO_WALL_BEHIND = -0.540;

// Three kinds of tabletop objects that we consider, 
// i.e. A (Georgia coffee blue), B (Suntory white), C (Gebisu brown)
static const double A_RADIUS = 0.052/2.;
static const double A_HEIGHT = 0.105;// This height seems too low
static const double B_RADIUS = 0.065/2.;
static const double B_HEIGHT = 0.123;
static const double C_RADIUS = 0.065/2.;
static const double C_HEIGHT = 0.167;
static const double VASE_R = 0.075;
static const double VASE_X = 0.350;
static const double VASE_Y = 0.;

static const size_t UPRIGHT = 0;
static const size_t DOWN = 1;

static const size_t PIXEL_PER_M = 500; // means that 1m is equal to <PIXEL_PER_M> pixels
static const double DEG_PER_RAD = 57.2957795;

class VisionSensor
{
public:
VisionSensor(ros::NodeHandle nh):
  nh_(nh)
{
  collision_space_pub_ = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object can not be added in the collision_space by the environment_server
  
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  set_planning_scene_diff_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);

  ROS_INFO("sensor_vis: up and running");
}

~VisionSensor()
{
}

bool
sense_see_srv_handle(hiro_sensor::Sense::Request& req, hiro_sensor::Sense::Response& res)
{
  if(req.sensor_type != 1)
  {
    res.msg = "Sorry, your request is not appropriate here!";
    return true;
  }

  // Hardcode the sensed_objects_----------------------------------------------------------------------
  sense_static_object();
   
  get_messy_config(2, false);

  // Although this remains questionable, without it, published collision objects can not be seen in rviz
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene. Env can not be configured correctly");
  } 
  // End of: Hardcode the sensed_objects_ --------------------------------------------------------------
  
  res.msg = "sense_see: Succeeded";
  return true;
}

void
run()
{
  ros::Rate tf_bc_rate(15.);
  
  while( ros::ok() )
  {
    for( std::map<std::string,geometry_msgs::PoseStamped>::iterator iter = sensed_objects_.begin(); iter != sensed_objects_.end(); ++iter )
    {
      tf::Transform transform;
      
      transform.setOrigin( tf::Vector3(iter->second.pose.position.x, iter->second.pose.position.y, iter->second.pose.position.z) );
      transform.setRotation( tf::Quaternion(iter->second.pose.orientation.x, iter->second.pose.orientation.y, iter->second.pose.orientation.z,iter->second.pose.orientation.w) );

      object_tf_bc_.sendTransform( tf::StampedTransform(transform, ros::Time::now(), iter->second.header.frame_id, iter->first) );
    }
    
    ros::spinOnce();
    
    tf_bc_rate.sleep();
  } 
}

private:
ros::NodeHandle nh_;

std::map<std::string,geometry_msgs::PoseStamped> sensed_objects_;// Modified in sense_movable_object(),...

ros::Publisher collision_space_pub_;

tf::TransformBroadcaster object_tf_bc_;// Must be outside the  while() loop

ros::ServiceClient set_planning_scene_diff_client_;

bool
sense_static_object()
{
  arm_navigation_msgs::CollisionObject table;
  
  table.id = "table";
  table.header.frame_id = "/link_base";
  table.header.stamp = ros::Time::now();
  table.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  // The table
  arm_navigation_msgs::Shape table_shape;
  
  table_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  table_shape.dimensions.resize(2);
  table_shape.dimensions[0] = TABLE_RADIUS;
  table_shape.dimensions[1] = TABLE_THICKNESS;
  
  table.shapes.push_back(table_shape);
      
  geometry_msgs::Pose table_pose;
  
  table_pose.position.x = 0.;
  table_pose.position.y = 0.;
  table_pose.position.z = TABLE_HEIGHT;
  table_pose.orientation.x = 0;
  table_pose.orientation.y = 0;
  table_pose.orientation.z = 0;
  table_pose.orientation.w = 1;

  table.poses.push_back(table_pose);
  
  collision_space_pub_.publish(table);
  ROS_INFO("The table should have been published");
  
//  // THe wall behind
//  arm_navigation_msgs::CollisionObject wall;
//  
//  wall.id = "wall";
//  wall.header.frame_id = "/link_base";
//  wall.header.stamp = ros::Time::now();
//  wall.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
//  
//  arm_navigation_msgs::Shape wall_shape;
//  
//  wall_shape.type = arm_navigation_msgs::Shape::BOX;
//  wall_shape.dimensions.resize(3);
//  wall_shape.dimensions[0] = 0.100;// size_x
//  wall_shape.dimensions[1] = TABLE_RADIUS*2;// size_y
//  wall_shape.dimensions[2] = (TABLE_HEIGHT*2) + TABLE_THICKNESS;// size_z
//  
//  geometry_msgs::Pose wall_pose;
//  
//  wall_pose.position.x = DIST_TO_WALL_BEHIND;
//  wall_pose.position.y = 0.;
//  wall_pose.position.z = wall_shape.dimensions[2]/2;
//  wall_pose.orientation.x = 0;
//  wall_pose.orientation.y = 0;
//  wall_pose.orientation.z = 0;
//  wall_pose.orientation.w = 1;
//  
//  wall.shapes.push_back(wall_shape);
//  wall.poses.push_back(wall_pose);
//  
//  collision_space_pub_.publish(wall);
//  ROS_INFO("The wall should have been published");
  
//  // The vase 
//  arm_navigation_msgs::CollisionObject vase;
//  
//  vase.id = "vase";
//  vase.header.frame_id = "/link_base";
//  vase.header.stamp = ros::Time::now();
//  vase.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
//  
//  arm_navigation_msgs::Shape vase_shape;
//  
//  vase_shape.type = arm_navigation_msgs::Shape::CYLINDER;
//  vase_shape.dimensions.resize(2);
//  vase_shape.dimensions[0] = VASE_R;
//  vase_shape.dimensions[1] = 0.40;// The real size of the flower vase is 0.40m
//  
//  geometry_msgs::Pose vase_pose;
//  
//  vase_pose.position.x = VASE_X;
//  vase_pose.position.y = VASE_Y;
//  vase_pose.position.z = TABLE_HEIGHT + (TABLE_THICKNESS/2) + (vase_shape.dimensions[1]/2);
//  vase_pose.orientation.x = 0;
//  vase_pose.orientation.y = 0;
//  vase_pose.orientation.z = 0;
//  vase_pose.orientation.w = 1;
//  
//  vase.shapes.push_back(vase_shape);
//  vase.poses.push_back(vase_pose);
//  
//  collision_space_pub_.publish(vase);
//  ROS_INFO("The vase should have been published");
  
  return true;
}  

bool
sense_movable_object( const std::string& object_id, 
              const double& radius, const double& height,
              const double& x, const double& y, const double& z,
              const double& qx=0., const double& qy=0., const double& qz=0., const double& qw=1.
            )
{
  const std::string frame_id = "/table";
  ros::Time time_stamp = ros::Time::now();
  
  // For tf needs
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.stamp = time_stamp;
  pose_stamped.header.frame_id = frame_id;
  
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;  
  pose_stamped.pose.position.z = z;    
     
  pose_stamped.pose.orientation.x = qx;
  pose_stamped.pose.orientation.y = qy;
  pose_stamped.pose.orientation.z = qz;
  pose_stamped.pose.orientation.w = qw;
  
  // Registering this movable object  
  sensed_objects_[object_id] = pose_stamped;
  
  // For collision_object needs
  arm_navigation_msgs::CollisionObject object;
  
  object.id = object_id;
  object.header.frame_id = frame_id;
  object.header.stamp = time_stamp;
  object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape object_shape;
  
  object_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  object_shape.dimensions.resize(2);
  object_shape.dimensions[0] = radius;
  object_shape.dimensions[1] = height;
  
  geometry_msgs::Pose object_pose;
  
  object_pose.position.x = x;
  object_pose.position.y = y;
  object_pose.position.z = z;
  
  object_pose.orientation.x = qx;
  object_pose.orientation.y = qy;
  object_pose.orientation.z = qz;
  object_pose.orientation.w = qw;
  
  object.shapes.push_back(object_shape);
  object.poses.push_back(object_pose);
  
  collision_space_pub_.publish(object);
  ROS_INFO_STREAM( object_id << " should has been published at " << object_pose.position.x << ", " << object_pose.position.y << ", " << object_pose.position.z );
  
  return true;
}


bool
is_in_collision(cv::Mat* img, const double& x, const double& y, const size_t& pitch_sym, const double& yaw)
{
  // Get the initial number of contour
  cv::Mat img_bin;
  const double th_bin = .5;
  cv::threshold(*img, img_bin, 255*th_bin, 255, cv::THRESH_BINARY_INV);

  std::vector< std::vector<cv::Point> > contours;
  cv::findContours( img_bin, 
	                  contours, // a vector of contours 
	                  CV_RETR_EXTERNAL,
	                  CV_CHAIN_APPROX_NONE  // retrieve all pixels of each contours
                  );

//  ROS_INFO_STREAM("contours.size()= " << contours.size());
  
  // Draw the new object
  cv::Mat img2;
  img2 = img->clone();
  
  cv::Point2f p;
  p.x = (x*PIXEL_PER_M) + img2.rows/2;
  p.y = (y*PIXEL_PER_M) + img2.cols/2;
  
  if(pitch_sym == UPRIGHT)
  {
    cv::circle( img2, p, B_RADIUS*PIXEL_PER_M, cv::Scalar(0), -1, CV_AA);
  }
  else
  {
    cv::Size2f size;
    size.width = 2*B_RADIUS*PIXEL_PER_M;
    size.height = B_HEIGHT*PIXEL_PER_M;
    
    cv::RotatedRect rect = cv::RotatedRect(p, size, yaw);
    
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (size_t i = 0; i < 4; i++)
      cv::line(img2, vertices[i], vertices[(i+1)%4], cv::Scalar(0), 1, CV_AA);
  }  
  
  // Get the final number of contour
  cv::Mat img_bin2;
  cv::threshold(img2, img_bin2, 255*th_bin, 255, cv::THRESH_BINARY_INV);

  std::vector< std::vector<cv::Point> > contours_f;
  cv::findContours( img_bin2, 
	                  contours_f, // a vector of contours 
	                  CV_RETR_EXTERNAL,
	                  CV_CHAIN_APPROX_NONE  // retrieve all pixels of each contours
                  );

//  cv::Mat img_con;
//  cv::cvtColor(img2, img_con, CV_GRAY2BGR, 3);
//  
//  cv::drawContours( img_con, 
//                    contours_f,
//                    -1, // negative means to draw all contours, which is indeed only one here
//                    CV_RGB(255, 0, 0),
//                    1 // with a thickness of 2 if positive
//                  );

//  ROS_INFO_STREAM("contours_f.size()= " << contours_f.size());
//  cv::imshow("img_con", img_con);
//  cv::waitKey(0);
  
  // Judge!
  if( contours_f.size() <= contours.size() )
  {
    return true;
  }
  else
  {
    *img = img2.clone();
    return false;
  }
}
//! Generate messy configs
/*!
  Assume that all objects are cylindrical: radius and height.
  Assume that orientation can be divided into 2 main groups: standing up-right and lying-down, for the latter there is an infiniti number of possible orientation.
*/
void
get_messy_config(const size_t& n, bool random=true)
{
  if( !random )
    return hardcode_messy_cfg(n);

  std::vector<std::string> ids;
  std::string common_id = "CAN";
  for(size_t i=0; i<n; ++i)
    ids.push_back( std::string(common_id+boost::lexical_cast<std::string>(i+1)) );

  // TODO how to make sure that no object is in collision with the others ?
  // Workaround: use opencv, check in 2D, the projection of objects
  
  boost::mt19937 rng( static_cast<unsigned int>(std::time(0)) );
  
  // Set the base 2D img for collision checking
  const size_t rows = (2*TABLE_RADIUS*PIXEL_PER_M); 
  const size_t cols = (2*TABLE_RADIUS*PIXEL_PER_M);
  
  cv::Mat img( rows, cols, CV_8UC1, cv::Scalar(255));// White image single channel 8 Unsigned
  
  // Set the dead zone: robot's torso + Set the unmovable_object (obstacles)
  const double rbt_r = 0.150;
  
  cv::Point rbt_p;
  rbt_p.x = (rows/2);
  rbt_p.y = (cols/2);

  cv::circle( img, rbt_p, rbt_r*PIXEL_PER_M, cv::Scalar(0), -1, CV_AA );  

//  const double vase_r = VASE_R;
//  
//  cv::Point vase_p;
//  vase_p.x = (VASE_X*PIXEL_PER_M) + (rows/2);
//  vase_p.y = (VASE_Y*PIXEL_PER_M) + (rows/2);

//  cv::circle( img, vase_p, vase_r*PIXEL_PER_M, cv::Scalar(0), -1, CV_AA );
  
  for(std::vector<std::string>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
  {
    std::string id = *i;
    const double r = B_RADIUS;
    const double h = B_HEIGHT;
    
    // Randomize the orientation
    const double roll = 0.;// Because the object is cylindrical, rolling affects nothing.
    
    boost::uniform_int<> uni_int_dist(0,1); 
    const size_t pitch_sym = uni_int_dist(rng);
    
    double pitch;
    if(pitch_sym == UPRIGHT)
      pitch = 0.;
    else
      pitch = M_PI/2;
    
    
    boost::uniform_real<double> uni_real_dist(0.,2*M_PI);
    const double yaw = uni_real_dist(rng) * DEG_PER_RAD;
    
    const tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    // Randomize the position.
    double z;
    if(pitch == 0.)
     z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);
    else
     z = (TABLE_THICKNESS/2)+(B_RADIUS);
     
    
    const double x_max = TABLE_RADIUS;// Play safe!
    const double x_min = -1 * x_max;
    const double y_max = x_max;
    const double y_min = x_min;
    boost::uniform_real<double> uni_real_dist_x(x_min, x_max);
    boost::uniform_real<double> uni_real_dist_y(y_min, y_max);
    
    double x, y;
    while( true and ros::ok() )
    {
      x = uni_real_dist_x(rng);
      y = uni_real_dist_y(rng);
      
      double r_here;
      r_here = sqrt( pow(x,2)+pow(y,2) );
      
      if( (r_here < (TABLE_RADIUS-B_HEIGHT)) and !is_in_collision(&img, x, y, pitch_sym, yaw) )
      {
        break;
      }
    }
    
    // Spawn the object    
    sense_movable_object( id, r, h, x, y, z, q.x(), q.y(), q.z(), q.w() );
  }
  
  // Draw the table boundary + save img for debugging purpose. 
  // Note that the object poses are mirrored; the robot is facing to the right of the img
  cv::Point tbl_p;
  tbl_p.x = (rows/2);
  tbl_p.y = (cols/2);
  cv::circle( img, tbl_p, TABLE_RADIUS*PIXEL_PER_M, cv::Scalar(0), 1, CV_AA );
 
//  cv::imwrite( "/home/vektor/4/ros-pkg/hiro_sensor/img/img.jpg", img );
//  cv::imshow("img", img);
//  cv::waitKey(0);
}
void
hardcode_messy_cfg(const size_t& n)
{
  switch(n)
  {
    case 4:
          sense_movable_object( "CAN4",
                                B_RADIUS, B_HEIGHT,
                                0.30, 0.25,(TABLE_THICKNESS/2)+(B_RADIUS),
                                0.,sqrt(0.5),0.,sqrt(0.5)
                              );
    case 3:
          sense_movable_object( "CAN3",
                                B_RADIUS, B_HEIGHT,
                                0.15,-0.40,(TABLE_THICKNESS/2)+(B_RADIUS),
                                0.,sqrt(0.5),0.,sqrt(0.5)
                              );
    case 2:
          sense_movable_object( "CAN2",
                                B_RADIUS, B_HEIGHT,
                                0.40, 0.25,(TABLE_THICKNESS/2)+(B_HEIGHT/2),
                                0.,0.,0.,1.
                              );
    case 1:
          sense_movable_object( "CAN1",
                                B_RADIUS, B_HEIGHT,
                                0.15, -0.30,(TABLE_THICKNESS/2)+(B_HEIGHT/2),
                                0.,0.,0.,1.
                              );
    case 0:
          ROS_WARN("There is no sensed object.");
  }
}
};// enf of: class VisionSensor

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_sensor");
  ros::NodeHandle nh;
  
  VisionSensor vis_sensor(nh);
  
  ros::ServiceServer sense_see_srv = nh.advertiseService("sense_see", &VisionSensor::sense_see_srv_handle, &vis_sensor);
  
  vis_sensor.run();

  return 0;
};
