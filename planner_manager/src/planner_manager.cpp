#include "planner_manager.hpp"
#include "sym_planner_manager.hpp"
#include "astar_utils.hpp"

using namespace boost;
using namespace std;

//! A constructor.
/*!
  ...
*/
PlannerManager::PlannerManager(ros::NodeHandle& nh):
  nh_(nh)
{
  ros::service::waitForService("plan_grasp");
  
  data_path_= ".";
  if( !ros::param::get("/data_path", data_path_) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  set_tidy_config();
  
  ROS_INFO("PlannerManager: Up and Running ...");
}
//! A destructor.
/*!
  The destructor does nothing.
*/
PlannerManager::~PlannerManager()
{ }

void 
PlannerManager::collision_object_cb(const arm_navigation_msgs::CollisionObject::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->id.c_str());
  
  // Filter unmovable_object
  if( !strcmp(msg->id.c_str(), "table")
      or !strcmp(msg->id.c_str(), "wall")
      or !strcmp(msg->id.c_str(), "vase")
    )
  {
    return;
  }
  
  messy_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );
}

bool
PlannerManager::plan_srv_handle(planner_manager::Plan::Request& req, planner_manager::Plan::Response& res)
{
  return plan( &(res.man_plan) );
}

//! Plan manipulation plans.
/*!
  The output is optimal and symbollically-and-geometrically feasible.
  It is obvious that this planner manager does not interleave symbolic planning and geometric planning.
  Instead, it makes a call to the former first in order to obtain all symbollically feasible task plans, then validate each of them using geometric planning.
  This is the most straightforward way to do combined symbolic and geometric planning.
  It also employ the concept of task motion multigraph.
  Searching over the task plan space encoded in a tmm, 
  within which a geometric planner is called to validate each action whether it is symbollically feasible.
*/
bool
PlannerManager::plan(std::vector<trajectory_msgs::JointTrajectory>* man_plan)
{
  // Create task plan space encoded in a task motion graph
  SymbolicPlannerManager spm(nh_);
  
  if( !spm.plan(*this) )
  {
    ROS_ERROR("Can not construct TMM");
    return false;
  }
  else
  {
    ROS_DEBUG("TMM is constructed successfully");
  }
  
  boost::dynamic_properties tmm_dp;
  
  tmm_dp.property("vertex_id", get(vertex_name, tmm_));
  tmm_dp.property("label", get(edge_name, tmm_));
  tmm_dp.property("weight", get(edge_weight, tmm_));
  tmm_dp.property("jspace", get(edge_jspace, tmm_)); 
  
  std::string tmm_dot_path = data_path_ + "/vanilla_tmm.dot";  
  std::ifstream tmm_dot(tmm_dot_path.c_str());
  
  read_graphviz(tmm_dot, tmm_, tmm_dp, "vertex_id"); 
  
  // Mark the root and the goal vertex in the TMM
  boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
  for(tie(vi,vi_end) = vertices(tmm_); vi != vi_end; ++vi)
  {
    std::string name;
    name = get(vertex_name, tmm_, *vi);
    
    if( !strcmp(name.c_str(), "MessyHome") )
      tmm_root_ = *vi;
    else if( !strcmp(name.c_str(), "TidyHome") )
      tmm_goal_ = *vi;
  }
  
  // Search 
  GeometricPlannerManager gpm(this);
  std::vector<TMMVertex> sol_path(num_vertices(tmm_));
  std::vector<double> sol_path_cost(num_vertices(tmm_));
  
  ROS_DEBUG("Start seaching.");
  ROS_DEBUG_STREAM("num_edges(tmm_),Before= " << num_edges(tmm_));
  try 
  {
    astar_search( tmm_
                , tmm_root_
                , AstarHeuristics<TaskMotionMultigraph, double>(tmm_goal_)
                , visitor(AstarVisitor<TaskMotionMultigraph>(tmm_goal_,&gpm))
                . predecessor_map(&sol_path[0])
                . distance_map(&sol_path_cost[0])
                );

  }
  catch(FoundGoalSignal fgs) 
  {
    cerr << "GOAL_FOUND" << endl;
    
    std::list<TMMVertex> sol_path_list;
    for(TMMVertex v = tmm_goal_; ; v = sol_path[v]) 
    {
      sol_path_list.push_front(v);
      if(sol_path[v] == v)
        break;
    }
    
    std::vector<TMMEdge> sol_path_edges;
    TMMVertex s;// the vertex before *spi in the loop
    
    std::list<TMMVertex>::iterator spi = sol_path_list.begin();
    s = *spi;
    
    cout << "Solution Path (vertices)= " << get(vertex_name, tmm_, *spi);
    for(++spi; spi != sol_path_list.end(); ++spi)
    {
      cout << " -> " << get(vertex_name, tmm_, *spi);
      
      TMMEdge cheapest_e;
      double cheapest_w = 9999.;

      graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
      for(tie(oei,oei_end)=out_edges(s, tmm_); oei!=oei_end; ++oei)
      {
        if( (target(*oei,tmm_)==*spi) and (get(edge_weight,tmm_,*oei) < cheapest_w) )
        {
          cheapest_e = *oei;
          cheapest_w = get(edge_weight,tmm_,*oei);
        }
      }
      
      sol_path_edges.push_back(cheapest_e);
      s = *spi;
    }
    cout << endl;
    
    cout << "Solution Path (edges)= " << endl;
    for(std::vector<TMMEdge>::iterator i=sol_path_edges.begin(); i!=sol_path_edges.end(); ++i)
    {
      put(edge_color,tmm_,*i,std::string("blue"));
      
      put( vertex_color,tmm_,source(*i,tmm_),color_traits<boost::default_color_type>::gray() );// for vertex in the solution path
      put( vertex_color,tmm_,target(*i,tmm_),color_traits<boost::default_color_type>::gray() );// somewhat redundant indeed
      
      man_plan->push_back( get(edge_plan,tmm_,*i) );
      
      cout << get(edge_name,tmm_,*i) << "[" << get(edge_jspace,tmm_,*i) << "]" << endl;
    }
    cout << endl;
    
    cout << "Sol Path Cost= " << sol_path_cost[tmm_goal_] << endl;
  }
  ROS_DEBUG_STREAM("num_edges(tmm_),After= " << num_edges(tmm_));
  
  // Write a fancy planned TMM
  std::string p_tmm_dot_path = data_path_ + "/fancy_tmm.dot";
  
  ofstream p_tmm_dot;
  p_tmm_dot.open(p_tmm_dot_path.c_str());
  
  write_graphviz( p_tmm_dot, tmm_
                , TMMVertexPropWriter<TMMVertexNameMap,TMMVertexColorMap>( get(vertex_name,tmm_),get(vertex_color,tmm_) )
                , TMMEdgePropWriter<TMMEdgeNameMap,TMMEdgeWeightMap,TMMEdgeJspaceMap,TMMEdgeColorMap>( get(edge_name,tmm_),get(edge_weight,tmm_),get(edge_jspace,tmm_),get(edge_color,tmm_) )
                );  
  
  p_tmm_dot.close();
  
  // Write a simplistic planned TMM for data collection
  boost::dynamic_properties p_tmm_dp;
  
  p_tmm_dp.property( "vertex_id",get(vertex_name,tmm_) );
  p_tmm_dp.property( "label",get(edge_name, tmm_) );
  p_tmm_dp.property( "weight",get(edge_weight, tmm_) );
  p_tmm_dp.property( "jspace",get(edge_jspace, tmm_) );
  p_tmm_dp.property( "color",get(edge_color, tmm_) );
  p_tmm_dp.property( "srcstate",get(edge_srcstate,tmm_) );

  std::string simple_p_tmm_dot_path = data_path_ + "/tmm.dot";
  ofstream simple_p_tmm_dot;
  simple_p_tmm_dot.open(simple_p_tmm_dot_path.c_str());
      
  write_graphviz_dp( simple_p_tmm_dot, tmm_, p_tmm_dp, std::string("vertex_id"));
  simple_p_tmm_dot.close();
  
  // Filter then write solution tmm
  SolEdgeFilter<TMMEdgeColorMap> sol_edge_filter( get(edge_color, tmm_) );
  typedef filtered_graph< TaskMotionMultigraph, SolEdgeFilter<TMMEdgeColorMap> > SolTMM;

  SolTMM sol_tmm(tmm_, sol_edge_filter);
  
  boost::dynamic_properties sol_tmm_dp;
  
  sol_tmm_dp.property( "vertex_id",get(vertex_name,sol_tmm) );
  sol_tmm_dp.property( "label",get(edge_name, sol_tmm) );
  sol_tmm_dp.property( "jspace",get(edge_jspace, sol_tmm) );
  sol_tmm_dp.property( "planstr",get(edge_planstr, sol_tmm) );  

  std::string sol_tmm_dot_path = data_path_ + "/sol_tmm.dot";
  ofstream sol_tmm_dot;
  sol_tmm_dot.open(sol_tmm_dot_path.c_str());
    
  write_graphviz_dp( sol_tmm_dot, sol_tmm, sol_tmm_dp, std::string("vertex_id"));
  sol_tmm_dot.close();
  
  return true;
}

//! Set the tidy_config_
/*!
  More...
  TODO should be from a file
*/
void
PlannerManager::set_tidy_config()
{
  const double B_RADIUS = 0.065/2.;
  const double B_HEIGHT = 0.123;
  const double TABLE_THICKNESS = 0.050;
  
  //------------------------------------------------------------------CAN1
  arm_navigation_msgs::CollisionObject can1;
   
  can1.id = "CAN1"; 

  can1.header.seq = 1;
  can1.header.stamp = ros::Time::now();
  can1.header.frame_id = "/table";
  can1.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  arm_navigation_msgs::Shape can1_shape;
  
  can1_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can1_shape.dimensions.resize(2);
  can1_shape.dimensions[0] = B_RADIUS;
  can1_shape.dimensions[1] = B_HEIGHT;
    
  can1.shapes.push_back(can1_shape);

  geometry_msgs::Pose can1_tidy_pose;
  
  can1_tidy_pose.position.x = 0.; 
  can1_tidy_pose.position.y = 0.35;
  can1_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can1_tidy_pose.orientation.x = 0.;    
  can1_tidy_pose.orientation.y = 0.;    
  can1_tidy_pose.orientation.z = 0.;    
  can1_tidy_pose.orientation.w = 1.;    
      
  can1.poses.push_back(can1_tidy_pose);
  
  tidy_cfg_[can1.id] = can1;
  
  //------------------------------------------------------------------------------CAN2
  arm_navigation_msgs::CollisionObject can2;
   
  can2.id = "CAN2"; 

  can2.header.seq = 1;
  can2.header.stamp = ros::Time::now();
  can2.header.frame_id = "/table";
  can2.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can2_shape;
  
  can2_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can2_shape.dimensions.resize(2);
  can2_shape.dimensions[0] = B_RADIUS;
  can2_shape.dimensions[1] = B_HEIGHT;
    
  can2.shapes.push_back(can2_shape);

  geometry_msgs::Pose can2_tidy_pose;
  
  can2_tidy_pose.position.x = 0.; 
  can2_tidy_pose.position.y = 0.42;
  can2_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can2_tidy_pose.orientation.x = 0.;    
  can2_tidy_pose.orientation.y = 0.;    
  can2_tidy_pose.orientation.z = 0.;    
  can2_tidy_pose.orientation.w = 1.;    
      
  can2.poses.push_back(can2_tidy_pose);
  
  tidy_cfg_[can2.id] = can2;  
  //------------------------------------------------------------------------------CAN3
  arm_navigation_msgs::CollisionObject can3;
   
  can3.id = "CAN3"; 

  can3.header.seq = 1;
  can3.header.stamp = ros::Time::now();
  can3.header.frame_id = "/table";
  can3.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can3_shape;
  
  can3_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can3_shape.dimensions.resize(2);
  can3_shape.dimensions[0] = B_RADIUS;
  can3_shape.dimensions[1] = B_HEIGHT;
    
  can3.shapes.push_back(can3_shape);

  geometry_msgs::Pose can3_tidy_pose;
  
  can3_tidy_pose.position.x = 0.; 
  can3_tidy_pose.position.y = 0.49;
  can3_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can3_tidy_pose.orientation.x = 0.;    
  can3_tidy_pose.orientation.y = 0.;    
  can3_tidy_pose.orientation.z = 0.;    
  can3_tidy_pose.orientation.w = 1.;    
      
  can3.poses.push_back(can3_tidy_pose);
  
  tidy_cfg_[can3.id] = can3;  
  //------------------------------------------------------------------------------CAN4
  arm_navigation_msgs::CollisionObject can4;
   
  can4.id = "CAN4"; 

  can4.header.seq = 1;
  can4.header.stamp = ros::Time::now();
  can4.header.frame_id = "/table";
  can4.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can4_shape;
  
  can4_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can4_shape.dimensions.resize(2);
  can4_shape.dimensions[0] = B_RADIUS;
  can4_shape.dimensions[1] = B_HEIGHT;
    
  can4.shapes.push_back(can4_shape);

  geometry_msgs::Pose can4_tidy_pose;
  
  can4_tidy_pose.position.x = -0.07; 
  can4_tidy_pose.position.y = 0.35;
  can4_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can4_tidy_pose.orientation.x = 0.;    
  can4_tidy_pose.orientation.y = 0.;    
  can4_tidy_pose.orientation.z = 0.;    
  can4_tidy_pose.orientation.w = 1.;    
      
  can4.poses.push_back(can4_tidy_pose);
  
  tidy_cfg_[can4.id] = can4;
  //------------------------------------------------------------------------------CAN5 TODO !rearrange!
  arm_navigation_msgs::CollisionObject can5;
   
  can5.id = "CAN5"; 

  can5.header.seq = 1;
  can5.header.stamp = ros::Time::now();
  can5.header.frame_id = "/table";
  can5.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can5_shape;
  
  can5_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can5_shape.dimensions.resize(2);
  can5_shape.dimensions[0] = B_RADIUS;
  can5_shape.dimensions[1] = B_HEIGHT;
    
  can5.shapes.push_back(can5_shape);

  geometry_msgs::Pose can5_tidy_pose;
  
  can5_tidy_pose.position.x = 0.14; 
  can5_tidy_pose.position.y = 0.42;
  can5_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can5_tidy_pose.orientation.x = 0.;    
  can5_tidy_pose.orientation.y = 0.;    
  can5_tidy_pose.orientation.z = 0.;    
  can5_tidy_pose.orientation.w = 1.;    
      
  can5.poses.push_back(can5_tidy_pose);

  tidy_cfg_[can5.id] = can5;
  //------------------------------------------------------------------------------CAN6
  arm_navigation_msgs::CollisionObject can6;
   
  can6.id = "CAN6"; 

  can6.header.seq = 1;
  can6.header.stamp = ros::Time::now();
  can6.header.frame_id = "/table";
  can6.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  arm_navigation_msgs::Shape can6_shape;
  
  can6_shape.type = arm_navigation_msgs::Shape::CYLINDER;
  can6_shape.dimensions.resize(2);
  can6_shape.dimensions[0] = B_RADIUS;
  can6_shape.dimensions[1] = B_HEIGHT;
    
  can6.shapes.push_back(can6_shape);

  geometry_msgs::Pose can6_tidy_pose;
  
  can6_tidy_pose.position.x = -0.21; 
  can6_tidy_pose.position.y = 0.49;
  can6_tidy_pose.position.z = (TABLE_THICKNESS/2)+(B_HEIGHT/2);  
  can6_tidy_pose.orientation.x = 0.;    
  can6_tidy_pose.orientation.y = 0.;    
  can6_tidy_pose.orientation.z = 0.;    
  can6_tidy_pose.orientation.w = 1.;    
      
  can6.poses.push_back(can6_tidy_pose);
  
  tidy_cfg_[can6.id] = can6; 
}

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_mgr");
  ros::NodeHandle nh;
 
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
 
  PlannerManager pm(nh);

  ros::Subscriber collision_object_sub;
  collision_object_sub = nh.subscribe("collision_object", 10, &PlannerManager::collision_object_cb, &pm);

  ros::ServiceServer plan_srv;
  plan_srv = nh.advertiseService("/plan", &PlannerManager::plan_srv_handle, &pm);

  ros::spin();
  
  return 0;
}
