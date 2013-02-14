#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//! First method
/*!
  Ref: http://www.cse.cuhk.edu.hk/~ttwong/papers/udpoint/udpoints.html
*/
void 
hammersley(const size_t& n,std::vector<geometry_msgs::Point>* points)
{
  for(size_t k=0; k<n; ++k)
  {
    double t = 0.;
    
    double p =0.5;
    for (size_t kk=k; kk; p*=0.5, kk>>=1)
      if (kk & 1) // kk mod 2 == 1
        t += p;
        
    t = 2.0 * t - 1.0; // map from [0,1] to [-1,1]
    
    double phi;
    phi = (k + 0.5) / n; // a slight shift
    
    double phirad;
    phirad = phi * 2.0 * M_PI; // map to [0, 2 pi)
    
    double st;
    st = sqrt(1.0-t*t);
    
    geometry_msgs::Point point;
    point.x = st * cos(phirad);
    point.y = st * sin(phirad);
    point.z = t;
    
    points->push_back(point);
  }
}

void
saff_spiral(std::vector<geometry_msgs::Point>* points,const size_t& n,const geometry_msgs::Point& center,double radius=1.0)
{
  const double C = 3.6;
  
  double pre_phi = 0.;// equals phi at k=1 and k=n
  for(size_t k=1; k<=n; ++k)
  {
    double h;
    h = -1. + ( (2.*(k-1))/(n-1) );
    
    double theta;
    theta = acos(h);
    
    double phi;
    if(k==1 or k==n)
      phi = 0.;
    else
      phi = (  pre_phi + ( (C/sqrt(n))*(1./sqrt(1.-pow(h,2))) )  );
      
    if(phi > (2.0*M_PI))
      phi = phi - (2.0*M_PI);
      
    pre_phi = phi;
    
    geometry_msgs::Point point;
    point.x = radius * sin(theta) * cos(phi) + center.x;
    point.y = radius * sin(theta) * sin(phi) + center.y;
    point.z = radius * cos(theta) + center.z;
    
    points->push_back(point);
  }
}

void
convert(double theta,double phi)
{
  double beta;//pitch
  beta = atan2( sin(theta),sqrt(pow(cos(theta)*cos(phi),2)+pow(sin(theta)*cos(phi),2)) );
  std::cerr << "beta= " << beta << std::endl;
  
  double alpha;
  alpha = atan2( sin(theta)*cos(phi)/cos(beta),cos(theta)*cos(phi)/cos(beta) );
  std::cerr << "alpha= " << alpha << std::endl;
  
  double gamma;
  gamma = atan2(0.,cos(phi)/cos(beta));
  std::cerr << "gamma= " << gamma << std::endl;
    
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ROS_INFO("heloo");
  ROS_DEBUG("debug");
  ros::spin();
  
  while (ros::ok())
  {
    // The sphere ============================================================================
    visualization_msgs::Marker sphere;
    
    double radius = 0.50;
    
    geometry_msgs::Point center;
    center.x = 0.;
    center.y = 0.;
    center.z = 0.;
        
    sphere.header.frame_id = "/table";
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "spheres";
    sphere.id = 0;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    // Set the pose of the sphere.  This is a full 6DOF pose relative to the frame/time specified in the header
    sphere.pose.position.x = center.x;
    sphere.pose.position.y = center.y;
    sphere.pose.position.z = center.z;
    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;

    // Set the scale of the sphere -- 1x1x1 here means 1m on a side
    sphere.scale.x = 2*radius;
    sphere.scale.y = 2*radius;
    sphere.scale.z = 2*radius;

    // Set the color -- be sure to set alpha to something non-zero!
    sphere.color.r = 0.0f;
    sphere.color.g = 1.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 0.5;

    sphere.lifetime = ros::Duration();

    // Publish the sphere
    marker_pub.publish(sphere);
    
    // Points on the sphere ============================================================================
    visualization_msgs::Marker points;
    
    points.header.frame_id = "/table";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    
    points.scale.x = 0.001;
    points.scale.y = 0.001;
    
    points.color.r = 1.0f;
    points.color.a = 1.0;

    // Distribute points uniformly in a sphere
    saff_spiral(&(points.points),10,center,radius);

    marker_pub.publish(points);
    
    // TF
    for(size_t i=0;i<points.points.size(); ++i)
{
    geometry_msgs::Point p;
    p = points.points.at(i);
    std::cerr << "p= [" << p.x << "," << p.y << "," << p.z << std::endl;
    
    double theta = acos(p.z/radius);
    double phi = atan2(p.y,p.x);
    
    std::cerr << "theta= " << theta << "= " << theta*180/M_PI << std::endl;
    std::cerr << "phi= " <<  phi << "= "<< phi*180/M_PI << std::endl;
    
//    tf::Vector3 r_vec;
//    r_vec = tf::Vector3(p.x,p.y,p.z);
//    r_vec.normalize();
//    
//    tf::Vector3 z_axis;
//    z_axis = tf::Vector3(0.,0.,1.);

//    tf::Quaternion q;    
//    q.setRotation(z_axis.cross(r_vec),phi);
////    q.setRotation(r_vec.cross(z_axis),phi);
//    
//    tf::Transform eof_tf;
//    eof_tf = tf::Transform( tf::createIdentityQuaternion(),tf::Vector3(p.x,p.y,p.z) );
//    
//    tf::Transform eof_tf_2;
//    eof_tf_2 = tf::Transform( q,tf::Vector3(0.,0.,0.) );

    static tf::TransformBroadcaster tf_bc;

    tf::Transform eof_tf;
    eof_tf = tf::Transform( tf::createQuaternionFromRPY(0.,0.,0.),tf::Vector3(p.x,p.y,p.z) );
    tf_bc.sendTransform( tf::StampedTransform(eof_tf, ros::Time::now(), "/table", "/eof_tf") );
    
    //==============================
    // rotate/tf_B around Z-axis-of-/table to align X-axis-of-/B with the projection of R
    tf::Transform tf_B;
    tf_B = tf::Transform( tf::createQuaternionFromRPY(0, 0.,phi),tf::Vector3(0.,0.,0.) );
    tf_bc.sendTransform( tf::StampedTransform(tf_B, ros::Time::now(), "/table", "/tf_B") );
    
    // rotate /tf_C around Y-axis-of-/tf_B to align Z-axis-of-/C with R
    tf::Transform tf_C;
    tf_C = tf::Transform( tf::createQuaternionFromRPY(0, theta,0.),tf::Vector3(0.,0.,0.) );
    tf_bc.sendTransform( tf::StampedTransform(tf_C, ros::Time::now(), "/tf_B", "/tf_C") );
    
    // rotate to adapt hiro-hand frame
    tf::Transform tf_D;
    tf_D = tf::Transform( tf::createQuaternionFromRPY(-1*M_PI/2, 0.,0.),tf::Vector3(0.,0.,0.) );
    tf_bc.sendTransform( tf::StampedTransform(tf_D, ros::Time::now(), "/tf_C", "/tf_D") );
    
    tf::Transform tf_rot_D_wrt_A;
    tf_rot_D_wrt_A = tf_B * tf_C * tf_D;
//    tf_bc.sendTransform( tf::StampedTransform(tf_rot_D_wrt_A, ros::Time::now(), "/table", "/tf_F") );

    tf::Transform tf_Z;
    tf_Z = tf::Transform( tf_rot_D_wrt_A.getRotation(),tf::Vector3(p.x,p.y,p.z) );
    tf_bc.sendTransform( tf::StampedTransform(tf_Z, ros::Time::now(), "/table", "/tf_Z") );
    
//    tf::StampedTransform transform;
//    try{
//      listener->waitForTransform("/table", "/tf_E",  now, ros::Duration(1.0));
//      listener->lookupTransform("/table", "/tf_E", now, transform);
//      
//      std::cerr << "tf= " << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << "," << std::endl;
//    }
//    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//    }
//            
//    convert(theta,phi);
//    convert(-1*theta,-1*phi);
r.sleep();
}   
//    r.sleep();
  }
}
