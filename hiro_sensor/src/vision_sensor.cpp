#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/PlanningScene.h>

#include "planning_environment/models/collision_models.h"
#include "planning_environment/models/model_utils.h"
#include "planning_environment/util/construct_object.h"

// This uses Boost 1.46.1 Library
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "hiro_sensor/Sense.h"
#include "hiro_sensor/See.h"

#include "utils.hpp"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

// TODO wrap this data in a header
// Based on observation in Jul 1, 2012
static const double TABLE_HEIGHT = 0.835;
static const double TABLE_RADIUS = 0.600;
static const double TABLE_THICKNESS = 0.050;
static const double DIST_TO_WALL_BEHIND = -0.540;

static const double VASE_R = 0.075;
static const double VASE_H = 0.400;
static const double VASE_X = 0.350;
static const double VASE_Y = 0.;

static const size_t UPRIGHT = 0;

static const size_t PIXEL_PER_M = 500; // means that 1m is equal to <PIXEL_PER_M> pixels
static const double DEG_PER_RAD = 57.2957795; // 1 radian = 57.2957795 degree

static boost::mt19937 g_cc_rng( static_cast<unsigned int>(std::time(0)) );

class VisionSensor
{
public:
VisionSensor(ros::NodeHandle nh)
:nh_(nh)
{
  collision_space_pub_ = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object can not be added in the collision_space by the environment_server
  
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  set_planning_scene_diff_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);
  
  ROS_INFO("sensor_vis: up and running");
}

~VisionSensor()
{ }

bool
see_srv_handle(hiro_sensor::See::Request& req, hiro_sensor::See::Response& res)
{
  // Hardcode the sensed_objects instead of from any vision sensor. Generate messy configs.
  if( !req.rerun )
  {
    init_cc_img();
    set_unmovable_obj_cfg(req.n_vase,req.randomized_vase);
    set_movable_obj_cfg(req.n_movable_object);
  }
  else// == rerun
  {
    set_obj_cfg(req.path);
  }
  
  // Write the randomized messy obj_cfg_
  std::string  data_path= ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  utils::write_obj_cfg( obj_cfg_,std::string(data_path+"/messy.cfg") );
  
  // Note that the object poses are mirrored because coordinate frame transformation is ignored: 
  // the robot is facing to the right of the img, the left side becomes the right side
  cv::imwrite( data_path+"/messy.cfg.jpg",cc_img_ );
   
  // Although this remains questionable, without it, published collision objects can not be seen in rviz
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

  if( !set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res) )
  {
    ROS_ERROR("Can't get planning scene. Env can not be configured correctly");
    return false;
  }
  
  return true;
}

//! This is publishing the tf for objects at forever till the ros is dead of this node is shut down
void
run()
{
  ros::Rate tf_bc_rate(15.);
  
  while( ros::ok() )
  {
    for( std::map<std::string,geometry_msgs::PoseStamped>::iterator iter = movable_obj_pose_map_.begin(); iter != movable_obj_pose_map_.end(); ++iter )
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
//! Set unmovable objs
/*!
  Unmovable objects are objects whose poses are fixed during planning, e.g. table, vase, wall, ...
  On most cases, they act as (unmovable) obstacles.
  
  Notice that some unmovable object configurations vary from one instance to another, e.g. vase objects
  
  \param n number of vases
*/
bool
set_unmovable_obj_cfg(const size_t& n,const bool& randomized)
{
  //---------------------------------------------------------------------------------------The table
  arm_navigation_msgs::CollisionObject table;
  
  table.id = "unmovable.table";
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
  
  spawn_object(table,&obj_cfg_);
//  //-------------------------------------------------------------------------------------- The wall behind
//  arm_navigation_msgs::CollisionObject wall;
//  
//  wall.id = "unmovable.wall";
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

//  spawn_object(wall,&obj_cfg_);

  //-------------------------------------------------------------------------------------- The vase(s)
  std::string common_id = "unmovable.VASE";

  // Randomize the poses and the shape of each vase
  for(size_t i=0; i<n; ++i) 
  {
    // Init
    std::string id = std::string( common_id+boost::lexical_cast<std::string>(i+1) );
    
    // Randomize
    double r,h;
    double x,y,z;
    tf::Quaternion q = tf::createQuaternionFromRPY(0.,0.,0.);// The orientation is fixed at UPRIGHT    
    
    if( !randomized )
    {
      r = VASE_R;
      h = VASE_H;
      
      x = VASE_X;
      y = VASE_Y;
      
      // Just to plot this vase in cc_img_
      is_in_collision(x,y,0.,0.,r,h,&cc_img_);
    }
    else
    {
      // Randomize the shape
      const double r_min = 0.030;
      const double r_max = 0.085;
      boost::uniform_real<double> uni_real_dist_r(r_min,r_max);
      
      r = uni_real_dist_r(g_cc_rng);

      const double h_min = 0.200;
      const double h_max = 0.500;
      boost::uniform_real<double> uni_real_dist_h(h_min,h_max);
      
      h = uni_real_dist_h(g_cc_rng);
      
      // Randomize the position: x, y
      const double x_max = TABLE_RADIUS;// Play safe!
      const double x_min = -1 * x_max;
      const double y_max = x_max;
      const double y_min = x_min;
      boost::uniform_real<double> uni_real_dist_x(x_min, x_max);
      boost::uniform_real<double> uni_real_dist_y(y_min, y_max);
      
      while( true and ros::ok() )
      {
        x = uni_real_dist_x(g_cc_rng);
        y = uni_real_dist_y(g_cc_rng);
        
        double r_here;
        r_here = sqrt( pow(x,2)+pow(y,2) );
        
        if( (r_here < (TABLE_RADIUS-utils::B_HEIGHT)) and !is_in_collision(x,y,0.,0.,r,h,&cc_img_) )
          break;
        
        // TODO 3D collision check between the vase and the robot
      }
    }
    
    z = (TABLE_THICKNESS/2)+(h/2);// because this vase is always UPRIGHT and frame_id = "/table"
    
    // Spawn the object to the planning environment
    spawn_object( id, std::string("/table"),
                  r, h, 
                  x, y, z, 
                  q.x(), q.y(), q.z(), q.w(),
                  &obj_cfg_ );
    
    if(!randomized) break;// only one vase can be spawned if not randomized
  }// end of: FOR each ordered VASE
 
  return true;
}  

//! set_movable_obj_cfg
/*!
  Assume that all objects are cylindrical: radius and height.
  Assume that orientation can be divided into 2 main groups: standing up-right and lying-down, for the latter there is an infiniti number of possible orientation.
*/
bool
set_movable_obj_cfg(const size_t& n)
{
  std::string common_id = "CAN";

  // Randomize the object pose
  for(size_t i=0; i<n; ++i)
  {
    std::string id = std::string(common_id+boost::lexical_cast<std::string>(i+1));
    const double r = utils::B_RADIUS;
    const double h = utils::B_HEIGHT;
    
    // Randomize the orientation
    const double roll = 0.;// Because the object is cylindrical, rolling affects nothing.
    
    boost::uniform_int<> uni_int_dist(0,1); 
    const size_t pitch_sym = uni_int_dist(g_cc_rng);
    
    double pitch;
    if(pitch_sym == UPRIGHT)
      pitch = 0.;
    else
      pitch = M_PI/2;
    
    boost::uniform_real<double> uni_real_dist(0.,2*M_PI);
    const double yaw = uni_real_dist(g_cc_rng) * DEG_PER_RAD;
    
    const tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    // Randomize the position.
    double z;
    if(pitch == 0.)
     z = (TABLE_THICKNESS/2)+(utils::B_HEIGHT/2);
    else
     z = (TABLE_THICKNESS/2)+(utils::B_RADIUS);
    
    const double x_max = TABLE_RADIUS;// Play safe!
    const double x_min = -1 * x_max;
    const double y_max = x_max;
    const double y_min = x_min;
    boost::uniform_real<double> uni_real_dist_x(x_min, x_max);
    boost::uniform_real<double> uni_real_dist_y(y_min, y_max);
    
    double x, y;
    while( true and ros::ok() )
    {
      x = uni_real_dist_x(g_cc_rng);
      y = uni_real_dist_y(g_cc_rng);
      
      double r_here;
      r_here = sqrt( pow(x,2)+pow(y,2) );
      
      if( (r_here < (TABLE_RADIUS-utils::B_HEIGHT)) and !is_in_collision(x,y,pitch,yaw,r,h,&cc_img_) )
      {
        break;
      }
    }
    
    // Spawn the object to the planning environment
    spawn_object( id, std::string("/table"),
                  r, h, 
                  x, y, z, 
                  q.x(), q.y(), q.z(), q.w(),
                  &obj_cfg_ );
  }

  return true;
}
//! Set both movable and unmovable object config from a given path
/*!
  More ...
*/
bool
set_obj_cfg(const std::string& path)
{
  utils::ObjCfg obj_cfg;
  
  // read movable+unmovable obj cfg
  if( !utils::read_obj_cfg(path,&obj_cfg) )
  {
    ROS_ERROR_STREAM("Can not find " << path);
    return false;
  }
  ROS_DEBUG_STREAM("obj_cfg.size()= " << obj_cfg.size());
 
  // Spawn objects in the planning environment
  for(utils::ObjCfg::iterator i=obj_cfg.begin(); i!=obj_cfg.end(); ++i)
    spawn_object(*i,&obj_cfg_);

  return true;
}

//! Spawn an (movable and or unmovable) object into the planning scene
/*!
  For movable objects, some registration for tf is also carried out.
*/
bool
spawn_object( const std::string& id, const std::string& frame,
              const double& radius, const double& height,
              const double& x, const double& y, const double& z,
              const double& qx, const double& qy, const double& qz, const double& qw,
              utils::ObjCfg* obj_cfg
             )
{
  ros::Time time_stamp;
  time_stamp = ros::Time::now();
  
  // Publish the object into planning environment
  arm_navigation_msgs::CollisionObject object;
  
  object.id = id;
  object.header.frame_id = frame;
  object.header.stamp = ros::Time::now();
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
  ROS_DEBUG_STREAM("Have published " << id << " (" << object_pose.position.x << "," << object_pose.position.y << "," << object_pose.position.z << ")");
 
  if(obj_cfg != NULL)
    obj_cfg->push_back(object);
  
  // Register for tf if the object is movable object
  std::vector<std::string> id_parts;// e.g. "unmovable.vase", "CAN1"
  boost::split( id_parts,id,boost::is_any_of(".") );
  
  std::string type;
  type = id_parts.at(0);
  
  if( !strcmp(type.c_str(),"unmovable") )
  {
    // do nothing!
  }
  else// type=="movable"
  {
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = time_stamp;
    pose_stamped.header.frame_id = frame;
    
    pose_stamped.pose.position.x = object.poses.at(0).position.x;
    pose_stamped.pose.position.y = object.poses.at(0).position.y;
    pose_stamped.pose.position.z = object.poses.at(0).position.z;
       
    pose_stamped.pose.orientation.x = object.poses.at(0).orientation.x;
    pose_stamped.pose.orientation.y = object.poses.at(0).orientation.y;
    pose_stamped.pose.orientation.z = object.poses.at(0).orientation.z;
    pose_stamped.pose.orientation.w = object.poses.at(0).orientation.w;
    
    // Registering this movable object's pose_stamped
    movable_obj_pose_map_[id] = pose_stamped;
  }
  
  return true;
}

bool
spawn_object(arm_navigation_msgs::CollisionObject object,utils::ObjCfg* obj_cfg)
{
  ros::Time time_stamp;
  time_stamp = ros::Time::now();
  
  // Publish the object into planning environment
  collision_space_pub_.publish(object);
  ROS_DEBUG_STREAM("Have published= " << object.id << " (" << object.poses.at(0).position.x << "," << object.poses.at(0).position.y << "," << object.poses.at(0).position.z << ")"); 
  
  if(obj_cfg != NULL)
    obj_cfg->push_back(object);
    
  // Register for tf if the object is movable object
  std::vector<std::string> id_parts;// e.g. "unmovable.vase", "CAN1"
  boost::split( id_parts,object.id,boost::is_any_of(".") );
  
  std::string type;
  type = id_parts.at(0);
  
  if( !strcmp(type.c_str(),"unmovable") )
  {
    // do nothing!
  }
  else// type=="movable"
  {
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = time_stamp;
    pose_stamped.header.frame_id = object.header.frame_id;
    
    pose_stamped.pose.position.x = object.poses.at(0).position.x;
    pose_stamped.pose.position.y = object.poses.at(0).position.y;
    pose_stamped.pose.position.z = object.poses.at(0).position.z;
       
    pose_stamped.pose.orientation.x = object.poses.at(0).orientation.x;
    pose_stamped.pose.orientation.y = object.poses.at(0).orientation.y;
    pose_stamped.pose.orientation.z = object.poses.at(0).orientation.z;
    pose_stamped.pose.orientation.w = object.poses.at(0).orientation.w;
    
    // Registering this movable object's pose_stamped
    movable_obj_pose_map_[object.id] = pose_stamped;
  } 
  
  return true;
}

bool
is_in_collision(const double& x,const double& y,const double& pitch,const double& yaw,const double& r,const double& h,cv::Mat* img)
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
  p.x = (x*PIXEL_PER_M) + img2.cols/2;
  p.y = (y*PIXEL_PER_M) + img2.rows/2;
  
  if(pitch == 0.)// == UPRIGHT
  {
    cv::circle( img2, p, r*PIXEL_PER_M, cv::Scalar(0), -1, CV_AA);
  }
  else
  {
    cv::Size2f size;
    size.width = 2*r*PIXEL_PER_M;
    size.height = h*PIXEL_PER_M;
    
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

//!init_cc_img();
/*!
  how to make sure that no object is in collision with the others ?
  Workaround: use opencv, check in 2D, the projection of objects
  
  This sets cc_img_
*/

void
init_cc_img()
{
  // Set the base 2D img for 2D image collision checking using opencv
  const size_t rows = (2*TABLE_RADIUS*PIXEL_PER_M); 
  const size_t cols = (2*TABLE_RADIUS*PIXEL_PER_M);
  
  cv::Mat img( rows, cols, CV_8UC1, cv::Scalar(255));// White image single channel 8 Unsigned
  cc_img_ = img;
  
  // Set the dead zone: robot's torso 
  const double rbt_r = 0.250;
  
  cv::Point rbt_p;
  rbt_p.x = (rows/2);
  rbt_p.y = (cols/2);
  
  cv::circle( cc_img_, rbt_p, rbt_r*PIXEL_PER_M, cv::Scalar(0), -1, CV_AA );
  
  // Draw the table boundary
  cv::Point tbl_p; 
  tbl_p.x = (rows/2); 
  tbl_p.y = (cols/2);
  
  cv::circle( cc_img_, tbl_p, TABLE_RADIUS*PIXEL_PER_M, cv::Scalar(0), 1, CV_AA );

}
 
ros::NodeHandle nh_;

//! Modified in sense_movable_object(),...
std::map<std::string,geometry_msgs::PoseStamped> movable_obj_pose_map_;

ros::Publisher collision_space_pub_;
tf::TransformBroadcaster object_tf_bc_;// Must be outside the  while() loop

ros::ServiceClient set_planning_scene_diff_client_;

//! Keeps all sense objects including both movable and unmovable objects
utils::ObjCfg obj_cfg_;

//! Keeps an image used for collision checking
cv::Mat cc_img_;
};// enf of: class VisionSensor

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_sensor");
  ros::NodeHandle nh;\
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    
  VisionSensor vis_sensor(nh);
  
  ros::ServiceServer see_srv = nh.advertiseService("/see", &VisionSensor::see_srv_handle, &vis_sensor);
  
  vis_sensor.run();

  return 0;
};
