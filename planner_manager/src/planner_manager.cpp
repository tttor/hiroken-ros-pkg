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
  ROS_DEBUG_STREAM("Heard= " << msg->id.c_str());
  
  // Filter unmovable_object
  if( !strcmp(msg->id.c_str(), "table")
      or !strcmp(msg->id.c_str(), "wall")
      or !strcmp(msg->id.c_str(), "vase")
      or !strcmp(msg->id.c_str(), "vase_2")
    )
  {
    return;
  }
  
  std::map<std::string, arm_navigation_msgs::CollisionObject>::iterator it;
  bool inserted;
  
  boost::tie(it,inserted) = messy_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );
  
  if(!inserted)
    it->second = *msg;// Update with the newer one
}

bool
PlannerManager::plan_srv_handle(planner_manager::Plan::Request& req, planner_manager::Plan::Response& res)
{
  return plan( req.mode,&(res.ctamp_sol) );
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
PlannerManager::plan(const size_t& mode,std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol)
{
  std::string base_data_path;
  if( !ros::param::get("/base_data_path", base_data_path) )
    ROS_WARN("Can not get /base_data_path, use the default value instead"); 
      
  std::string data_path;
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");

  std::string tidy_cfg_filename;
  if( !ros::param::get("/tidy_cfg_filename",tidy_cfg_filename) )
    ROS_WARN("Can not get /tidy_cfg_filename, use the default value instead");         

 if ( !set_tidy_config() )
  {
    ROS_ERROR("Can not set the tidy_cfg!");
    return false;
  }
      
  // Init
  switch(mode)
  {
    case 1:
    {
      tmm_ = TaskMotionMultigraph();// renew the tmm_
        
      // Create task plan space encoded in a task motion graph
      // A call to the task planner if succeed outputs "vanilla_tmm.dot"
      SymbolicPlannerManager spm(nh_);
      
      if( !spm.plan(*this) )
      {
        ROS_ERROR("Can not construct TMM!");
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
      
      std::string tmm_dot_path = data_path + "/vanilla_tmm.dot";  
      std::ifstream tmm_dot(tmm_dot_path.c_str());
      
      read_graphviz(tmm_dot, tmm_, tmm_dp, "vertex_id"); 
      
      break;
    }
    case 2:
    {
      // use same as mode=3
    }
    case 3:
    {
      // use same as mode=4
    }
    case 4:
    {
      tmm_ = TaskMotionMultigraph();// renew the tmm_
    
      // Read the planned tmm
      boost::dynamic_properties tmm_dp;
      
      tmm_dp.property("vertex_id", get(vertex_name, tmm_));
      
      tmm_dp.property("label", get(edge_name, tmm_));
      tmm_dp.property("weight", get(edge_weight, tmm_));
      tmm_dp.property("jspace", get(edge_jspace, tmm_)); 
      tmm_dp.property("color", get(edge_color,tmm_));
      tmm_dp.property("srcstate", get(edge_srcstate,tmm_));
      tmm_dp.property("mptime",get(edge_mptime,tmm_));
      tmm_dp.property("planstr",get(edge_planstr,tmm_));

      std::ifstream tmm_dot(  std::string(base_data_path+"/tmm.dot").c_str()  );
      if( !read_graphviz(tmm_dot, tmm_, tmm_dp, "vertex_id") )
      {
        ROS_ERROR("read_graphviz(): Failed.");
        return false;
      }
      
      // Put edge_planstr into edge_plan
      graph_traits<TaskMotionMultigraph>::edge_iterator ei,ei_end;
      for(tie(ei,ei_end)=edges(tmm_); ei!=ei_end; ++ei)
      {
        std::string planstr;
        planstr = get(edge_planstr,tmm_,*ei);
        
        put(edge_plan,tmm_,*ei, get_plan(planstr) );
      }
      
      // Somehow(?) within this mode astar doesnot call init_vertex()
      graph_traits<TaskMotionMultigraph>::vertex_iterator vi,vi_end;
      for(tie(vi,vi_end)=vertices(tmm_); vi!=vi_end; ++vi)
      {
        GeometricPlannerManager gpm(this);
        gpm.init_vertex(*vi);
      }
      
      break;
    }
  }
  
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
  
  // Open a log file
  std::ofstream perf_log;
  perf_log.open(std::string(data_path+"/perf.log").c_str());
  
  std::ofstream csv_perf_log;
  csv_perf_log.open(std::string(data_path+"/perf.log.csv").c_str());

  // Search 
  GeometricPlannerManager gpm(this);
  std::vector<TMMVertex> predecessors(num_vertices(tmm_));
  std::vector<double> distances(num_vertices(tmm_));
  
  ROS_DEBUG("Start searching over TMM.");
  ros::Time planning_begin = ros::Time::now();
  try 
  {
    switch(mode)
    {
      case 1:
      {
        // use "as is" mode2, the learner is useless though
      }
      case 2:
      {
        // SVR from libsvm
        std::string model_path;
        model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/svm.v4.libsvmmodel";
        
        std::string te_data_path;
        te_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/online_te_data.libsvmdata";

        std::string fit_out_path;
        fit_out_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/fit.out";
        
        SVM_Object learner(model_path,te_data_path,fit_out_path,"");
        
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,SVM_Object>(tmm_goal_,&gpm,&learner,mode)
                    , visitor( AstarVisitor<TaskMotionMultigraph,SVM_Object>(tmm_goal_,&gpm,&learner,mode) )
                    . predecessor_map(&predecessors[0])
                    . distance_map(&distances[0])
                    );
        
        break;
      }
      case 3:
      {
        // Use the same machine as mode=4 (LWPR) but the model is not updated online
      }
      case 4:
      {
        ROS_DEBUG("learner= LWPR");
        
        // LWPR from Edinburg Univ.
        std::string lwpr_model_path;
        lwpr_model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/lwpr.bin";
                
        LWPR_Object lwpr_learner(lwpr_model_path.c_str());
        
        ROS_DEBUG("Searching...");
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,LWPR_Object>(tmm_goal_,&gpm,&lwpr_learner,mode)
                    , visitor( AstarVisitor<TaskMotionMultigraph,LWPR_Object>(tmm_goal_,&gpm,&lwpr_learner,mode) )
                    . predecessor_map(&predecessors[0])
                    . distance_map(&distances[0])
                    );
            
        break;
      }
    }
  }
  catch(FoundGoalSignal fgs) 
  {
    ROS_INFO("GOAL_FOUND.");
    
    // Get elapsed time for planning
    double planning_time;
    planning_time = (ros::Time::now()-planning_begin).toSec();
    
    cout << "CTAMP_SearchTime= " << planning_time << endl;
    perf_log << "CTAMP_SearchTime=" << planning_time << endl;
    csv_perf_log << planning_time << ",";
    
    // Get #expanded vertices
    size_t n_expvert = 0;
    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;

    double expansion_time = 0.;
      
    for(boost::tie(vi,vi_end) = vertices(tmm_); vi!=vi_end; ++vi)
    {
      if(get(vertex_color,tmm_,*vi)==color_traits<boost::default_color_type>::black())
      {
        // Count #expanded vertices
        ++n_expvert;
        
        typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
        for(tie(oei,oei_end)=out_edges(*vi,tmm_); oei!=oei_end; ++oei)
        {
          expansion_time += get(edge_mptime,tmm_,*oei);
        }
      }
      else
      {
        // Reset and Make-sure that for out-edges of unexpanded vertices these values prevail
        typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
        for(tie(oei,oei_end)=out_edges(*vi,tmm_); oei!=oei_end; ++oei)
        {
          put(edge_color,tmm_,*oei,"black");
          put(edge_weight,tmm_,*oei,0.);
        }
      }
    }
    
    cout << "expansion_time= " << expansion_time << endl;
    perf_log << "expansion_time=" << expansion_time << endl;
    csv_perf_log << expansion_time << ",";
    
    cout << "#ExpandedVertices= " << n_expvert << endl;
    perf_log << "#ExpandedVertices=" << n_expvert << endl;
    csv_perf_log << n_expvert << ",";
    
    cout << "#Vertices= " << num_vertices(tmm_) << endl;
    perf_log << "#Vertices=" << num_vertices(tmm_) << endl;
    csv_perf_log << num_vertices(tmm_) << ",";
        
    // Convert from vector to list to enable push_front()
    std::list<TMMVertex> sol_path_vertex_list;
    for(TMMVertex v = tmm_goal_; ; v = predecessors[v]) 
    {
      sol_path_vertex_list.push_front(v);
      if(predecessors[v] == v)
        break;
    }
    
    // Build the solution path. Here the solution path may be cancelled if there is an edge that has no motion plan (edge_color=red).
    std::vector<TMMEdge> sol_path;
    bool recheck = true;// Optimistis that this is truly solution path
    std::list<TMMVertex>::iterator spvl_it = sol_path_vertex_list.begin();

    TMMVertex s;
    s = *spvl_it;
    
    cout << "SolutionPath(v)=" << endl;
    perf_log << "SolutionPath(v)=";
    
    cout << get(vertex_name, tmm_, *spvl_it) << endl;
    perf_log << get(vertex_name, tmm_, *spvl_it) << ",";
    
    for(++spvl_it; spvl_it != sol_path_vertex_list.end(); ++spvl_it)
    {
      cout << get(vertex_name, tmm_, *spvl_it) << endl;
      perf_log << get(vertex_name, tmm_, *spvl_it) << ",";
      
      // Get the cheapest path among many due to parallelism/multigraph
      TMMEdge cheapest_e;
      double cheapest_w = std::numeric_limits<double>::max();

      graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
      for(tie(oei,oei_end)=out_edges(s, tmm_); oei!=oei_end; ++oei)
      {
        if( (target(*oei,tmm_)==*spvl_it) and (get(edge_weight,tmm_,*oei) < cheapest_w) )
        {
          cheapest_e = *oei;
          cheapest_w = get(edge_weight,tmm_,*oei);
        }
      }
      
      sol_path.push_back(cheapest_e);
      s = *spvl_it;
    }
    cout << endl;
    perf_log << endl;
    
    cout << "SolutionPath(e)=" << endl;
    perf_log << "SolutionPath(e)=";
    
    for(std::vector<TMMEdge>::iterator i=sol_path.begin(); i!=sol_path.end(); ++i)
    {
      cout << get(edge_name,tmm_,*i) << "[" << get(edge_jspace,tmm_,*i) << "]" << endl;
      perf_log << get(edge_name,tmm_,*i) << "[" << get(edge_jspace,tmm_,*i) << "]" << ",";
      
      if( !strcmp(get(edge_color,tmm_,*i).c_str(),"red") )
      {
        // TODO may re-do motion planning here for this edge
      
        recheck = false;
      }
      else if( !strcmp(get(edge_color,tmm_,*i).c_str(),"green") )
      {
        put( edge_color,tmm_,*i,std::string("blue") );
        put( vertex_color,tmm_,source(*i,tmm_),color_traits<boost::default_color_type>::gray() );// for vertex in the solution path
        put( vertex_color,tmm_,target(*i,tmm_),color_traits<boost::default_color_type>::gray() );// somewhat redundant indeed
        
        ctamp_sol->push_back( get(edge_plan,tmm_,*i) );
      }
    }
    cout << endl;
    perf_log << endl;
    
    // TODO confirming step
    if(recheck)
    {
      cout << "SolPathCost= " << distances[tmm_goal_] << endl;
      perf_log << "SolPathCost=" << distances[tmm_goal_] << endl;
      csv_perf_log << distances[tmm_goal_];
    }
    else
    {
      // TODO bypass unimplementable edges, needs to replan the motion
      
      ctamp_sol->clear();
      
      cout << "SolPathCost= UNDEFINED" << endl;
      perf_log << "SolPathCost=UNDEFINED" << endl;
      
      cout << "SolPathAbove= CANCELLED" << endl;
      perf_log << "SolPathAbove=CANCELLED" << endl;
    }
  }// End of: catch(FoundGoalSignal fgs) 
  
  // Write a fancy planned TMM
  std::string p_tmm_dot_path = data_path + "/fancy_tmm.dot";
  
  ofstream p_tmm_dot;
  p_tmm_dot.open(p_tmm_dot_path.c_str());
  
  write_graphviz( p_tmm_dot, tmm_
                , TMMVertexPropWriter< TMMVertexNameMap,TMMVertexColorMap,TMMVertexHeuMap,std::vector<double> >( get(vertex_name,tmm_),get(vertex_color,tmm_),get(vertex_heu,tmm_),distances )
                , TMMEdgePropWriter<TMMEdgeNameMap,TMMEdgeWeightMap,TMMEdgeJspaceMap,TMMEdgeColorMap>( get(edge_name,tmm_),get(edge_weight,tmm_),get(edge_jspace,tmm_),get(edge_color,tmm_) )
                );  
  
  p_tmm_dot.close();
  
  // Write a simplistic planned TMM for later use, e.g. offline training
  boost::dynamic_properties p_tmm_dp;
  
  p_tmm_dp.property( "vertex_id",get(vertex_name,tmm_) );
  
  p_tmm_dp.property( "label",get(edge_name, tmm_) );
  p_tmm_dp.property( "weight",get(edge_weight, tmm_) );
  p_tmm_dp.property( "jspace",get(edge_jspace, tmm_) );
  p_tmm_dp.property( "color",get(edge_color, tmm_) );
  p_tmm_dp.property( "srcstate",get(edge_srcstate,tmm_) );
  p_tmm_dp.property( "mptime",get(edge_mptime,tmm_) );
  p_tmm_dp.property( "planstr",get(edge_planstr,tmm_) );  

  std::string simple_p_tmm_dot_path = data_path + "/tmm.dot";
  ofstream simple_p_tmm_dot;
  simple_p_tmm_dot.open(simple_p_tmm_dot_path.c_str());
      
  write_graphviz_dp( simple_p_tmm_dot, tmm_, p_tmm_dp, std::string("vertex_id"));
  simple_p_tmm_dot.close();
  
  // Filter then write the solution tmm
  SolEdgeFilter<TMMEdgeColorMap> sol_edge_filter( get(edge_color, tmm_) );
  typedef filtered_graph< TaskMotionMultigraph, SolEdgeFilter<TMMEdgeColorMap> > SolTMM;

  SolTMM sol_tmm(tmm_, sol_edge_filter);
  
  boost::dynamic_properties sol_tmm_dp;
  
  sol_tmm_dp.property( "vertex_id",get(vertex_name,sol_tmm) );
  sol_tmm_dp.property( "label",get(edge_name, sol_tmm) );
  sol_tmm_dp.property( "jspace",get(edge_jspace, sol_tmm) );
  sol_tmm_dp.property( "planstr",get(edge_planstr, sol_tmm) );  

  std::string sol_tmm_dot_path = data_path + "/sol_tmm.dot";
  ofstream sol_tmm_dot;
  sol_tmm_dot.open(sol_tmm_dot_path.c_str());
    
  write_graphviz_dp( sol_tmm_dot, sol_tmm, sol_tmm_dp, std::string("vertex_id"));
  sol_tmm_dot.close();
  
  // Addition info. for perf.log
  perf_log << "tidy.cfg=" << base_data_path << tidy_cfg_filename;
  
  perf_log.close();
  csv_perf_log.close();
  return true;
}

//! Set the tidy_config_
/*!
  More...
*/
bool
PlannerManager::set_tidy_config()
{
  ObjCfg tidy_cfg;
  
  // Read the given tidy_cfg under base_data_path
  std::string base_data_path;
  if( !ros::param::get("/base_data_path", base_data_path) )
    ROS_WARN("Can not get /base_data_path, use the default value instead");
    
  std::string tidy_cfg_filename;// The base_data_path is a constant
  if( !ros::param::get("/tidy_cfg_filename",tidy_cfg_filename) )
    ROS_WARN("Can not get /tidy_cfg_filename, use the default value instead"); 
    
  if( !read_obj_cfg(std::string(base_data_path+tidy_cfg_filename),&tidy_cfg) )
  {
    ROS_ERROR("Can not find the tidy*.cfg file.");
    return false;
  }
  
  for(ObjCfg::iterator i=tidy_cfg.begin(); i!=tidy_cfg.end(); ++i)
    tidy_cfg_[i->id] = *i; 

  // Write the randomized obj_cfg
  std::string  data_path= ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  write_obj_cfg( tidy_cfg,std::string(data_path+"/tidy.cfg") );
  
  return true;
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
  
  ROS_INFO("PlannerManager: Up and Running ...");
  ros::spin();
  
  return 0;
}
