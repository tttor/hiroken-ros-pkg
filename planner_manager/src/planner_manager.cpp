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

//! collision_object_cb
/*!
  Movable object id -> CAN1, CAN2. ..., CANn
  Unmovable object id -> unmovable.table, unmovable.vase, ...
*/
void 
PlannerManager::collision_object_cb(const arm_navigation_msgs::CollisionObject::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Heard= " << msg->id.c_str());
  
  // Filter out unmovable object
  std::vector<std::string> id_parts;// e.g. "unmovable.vase", "CAN1"
  boost::split( id_parts,msg->id,boost::is_any_of(".") );
  
  std::string type;
  type = id_parts.at(0);
  
  if( !strcmp(type.c_str(),"unmovable") )
  {
    return;
  }
  
  std::map<std::string, arm_navigation_msgs::CollisionObject>::iterator it;
  bool inserted;
  
  boost::tie(it,inserted) = movable_obj_messy_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );
  
  if(!inserted)
    it->second = *msg;// Update with the newer one
}

bool
PlannerManager::plan_srv_handle(planner_manager::Plan::Request& req, planner_manager::Plan::Response& res)
{
  return plan( req.ml_mode,req.rerun,req.log_path,&(res.ctamp_sol),&(res.ctamp_log) );
}

//! Plan manipulation plans.
/*!
  The output is asymptotically-optimal and symbollically-and-geometrically feasible.
  
  It is obvious that this planner manager does not interleave symbolic planning and geometric planning.
  Instead, it makes a call to the former first in order to obtain all symbollically feasible task plans, then validate each of them using geometric planning.
  This is the most straightforward way to do combined symbolic and geometric planning.
  It also employ the concept of task motion multigraph.
  Searching over the task plan space encoded in a tmm, 
  within which a geometric planner is called to validate each action whether it is symbollically feasible.
*/
bool
PlannerManager::plan(const size_t& ml_mode,const bool& rerun,const std::string& log_path,std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol,std::vector<double>* ctamp_log)
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
      
  // Initialize
  TaskMotionMultigraph ucs_sol_tmm;// only used in rerun modes
  
  if(!rerun)
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
    ROS_DEBUG("TMM is constructed successfully");
    
    boost::dynamic_properties tmm_dp;
    
    tmm_dp.property("vertex_id", get(vertex_name, tmm_));
    
    tmm_dp.property("label", get(edge_name, tmm_));
    tmm_dp.property("weight", get(edge_weight, tmm_));
    tmm_dp.property("jspace", get(edge_jspace, tmm_)); 
    
    std::string tmm_dot_path = data_path + "/vanilla_tmm.dot";  
    std::ifstream tmm_dot(tmm_dot_path.c_str());
    
    read_graphviz(tmm_dot, tmm_, tmm_dp, "vertex_id"); 
  }
  else// if(rerun)
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
      ROS_ERROR("read_graphviz(tmm_dot,...): Failed");
      return false;
    }
    ROS_DEBUG("read_graphviz(tmm_dot,...): Succeeded");
    
    // Put edge_planstr into edge_plan
    graph_traits<TaskMotionMultigraph>::edge_iterator ei,ei_end;
    for(tie(ei,ei_end)=edges(tmm_); ei!=ei_end; ++ei)
    {
      std::string planstr;
      planstr = get(edge_planstr,tmm_,*ei);
      
      put(edge_plan,tmm_,*ei, get_plan(planstr) );
    }
    
    // Retrieve the sol path resulted using UCS for making comparison: true_cost-to-go vs predicted_cost-to-go 
    // Assume that the base for rerun is using UCS
    boost::dynamic_properties ucs_sol_tmm_dp;
    
    ucs_sol_tmm_dp.property("vertex_id", get(vertex_name, ucs_sol_tmm));
    
    ucs_sol_tmm_dp.property("label", get(edge_name, ucs_sol_tmm));
    ucs_sol_tmm_dp.property("jspace", get(edge_jspace, ucs_sol_tmm)); 
    ucs_sol_tmm_dp.property("planstr",get(edge_planstr,ucs_sol_tmm));    
//    ucs_sol_tmm_dp.property("weight", get(edge_weight, ucs_sol_tmm));
//    ucs_sol_tmm_dp.property("color", get(edge_color,ucs_sol_tmm));

    std::ifstream ucs_sol_tmm_dot(  std::string(base_data_path+"/sol_tmm.dot").c_str()  );
    if( !read_graphviz(ucs_sol_tmm_dot, ucs_sol_tmm, ucs_sol_tmm_dp, "vertex_id") )
    {
      ROS_ERROR("read_graphviz(ucs_sol_tmm_dot,...): Failed.");
      return false;
    }
    
    // Copy the weight from tmm_ to ucs_sol_tmm because currently ucs_sol_tmm does not contain these values
    for(tie(ei,ei_end) = edges(ucs_sol_tmm); ei!=ei_end; ++ei )
    {
      graph_traits<TaskMotionMultigraph>::edge_iterator ej,ej_end;
      for(tie(ej,ej_end) = edges(tmm_); ej!=ej_end; ++ej)
      {
        TMMVertex ei_source = source(*ei,ucs_sol_tmm);
        TMMVertex ei_target = target(*ei,ucs_sol_tmm);
        
        TMMVertex ej_source = source(*ej,tmm_);
        TMMVertex ej_target = target(*ej,tmm_);
        
        if( (ei_source==ej_source) and (ei_target==ej_target) )
        {
          double w;
          w = get(edge_weight,tmm_,*ej);
          
          put(edge_weight,ucs_sol_tmm,*ei,w);
          
          break;
        }
      }
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
  
  // Open up log files
  std::ofstream perf_log;
  perf_log.open(std::string(data_path+"/perf.log").c_str());
  
  std::ofstream csv_perf_log;
  csv_perf_log.open(std::string(data_path+"/perf.log.csv").c_str());

  // ml-related data keeper
  std::vector< std::vector<double> > ml_data;

  std::string model_path;// Used only in SVR_OFFLINE mode
  model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/svm.libsvmmodel";
  
  // Search 
  GeometricPlannerManager gpm(this);
  std::vector<TMMVertex> predecessors(num_vertices(tmm_));
  std::vector<double> distances(num_vertices(tmm_));
  
  ros::Time planning_begin = ros::Time::now();
  try 
  {
    switch(ml_mode)
    {
      case NO_ML:
      {
        // use "as is" mode=SVR_OFFLINE, 
        //the learner is useless though, see astar_utils.hpp where the heuristic for this mode is always set to h=0
        ROS_DEBUG("learner= NO_ML but via learner= SVR");
      }
      case SVR_OFFLINE:
      {
        ROS_DEBUG("learner= SVR");
        
        // SVR from libsvm
        std::string te_data_path;// for prediction using SVM
        te_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/te_data.libsvmdata";

        std::string fit_out_path;// for prediction using SVM
        fit_out_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/fit.out";
        
        SVM_Object learner(model_path,te_data_path,fit_out_path,"");
        
        ROS_DEBUG("Searching over TMM ...");
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,SVM_Object>(tmm_goal_,&gpm,&learner,ml_mode)
                    , visitor( AstarVisitor<TaskMotionMultigraph,SVM_Object>(tmm_goal_,&gpm,&learner,&ml_data,ml_mode) )
                    . predecessor_map(&predecessors[0])
                    . distance_map(&distances[0])
                    );
        
        break;
      }
      case LWPR_ONLINE:
      {
        ROS_DEBUG("learner= LWPR");// LWPR from Edinburg Univ.
        
        std::string model_path;
        model_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/lwpr.bin";
                
        LWPR_Object learner( model_path.c_str() );
        
        ROS_DEBUG("Searching over TMM ...");
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,LWPR_Object>(tmm_goal_,&gpm,&learner,ml_mode)
                    , visitor( AstarVisitor<TaskMotionMultigraph,LWPR_Object>(tmm_goal_,&gpm,&learner,&ml_data,ml_mode) )
                    . predecessor_map(&predecessors[0])
                    . distance_map(&distances[0])
                    );

        break;
      }
      default:
      {
        ROS_ERROR("ml_mode is unknown, return immediately.");
        return false;
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
    bool recheck = true;// Optimistis that this is truly the solution path
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
  
  if(ml_mode==SVR_OFFLINE)
  {
    // Initialize
    std::string tr_data_path;// is appended with samples from one CTAMP instance to anothor
    tr_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/tr_data.libsvmdata";// _must_ be synch with the one in astar_utils.hpp
    
    std::string fit_data_path;// _must_ be always overwritten
    fit_data_path = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/fit.out";
        
//    // BENCHmark:Obtain testing data error: the different between the fitting and the true value of testing data (which have _not_ been used to train the model)
//    FILE *input;
//    input = fopen(tr_data_path.c_str(),"r");
//    if(input == NULL)
//    {
//      std::cerr << "can't open input file= " << tr_data_path << std::endl;
//      return false;
//    }
//    
//    FILE *output;
//    output = fopen(fit_data_path.c_str(),"w");// overwrite
//    if(output == NULL)
//    {
//      std::cerr << "can't open output file= " << fit_data_path << std::endl;
//      return false;
//    }

//    SVMModel* trained_model;// trained/updated using a bunch of data from seach for this instance
//    if( (trained_model=svm_load_model(model_path.c_str())) == 0 )
//    {
//      std::cerr << "can't open model file= " << model_path << std::endl;
//      return false;
//    }

//    SVMNode* x;
//    int max_nr_attr = 100;
//    
//    x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));

//    int predict_probability=0;
//    
//    libsvm_predict(trained_model,predict_probability,max_nr_attr,x,input,output);
//    
//    // TODO make these work, now: causes running-time ERR
////    svm_free_and_destroy_model(&model);
////    free(x);
//    fclose(input);
//    fclose(output);
//    
//    // Obtain y_fit from a tmp(hot) file, then put it in ml_data, 
//    // have to read fit.out file because libsvm_predict() returns the results in a file
//    std::string line;
//    
//    std::vector<double> fit_out_content;
//    std::ifstream fit_out_file(fit_data_path.c_str());
//    if(fit_out_file.is_open())
//    {
//      while ( fit_out_file.good() )
//      {
//        std::getline(fit_out_file,line);
//        if(line.size() != 0)
//          fit_out_content.push_back( boost::lexical_cast<double>(line) );
//      }
//      fit_out_file.close();
//    }
//    else
//    {
//     std::cout << "fit_out_file.is_open(): Failed" << std::endl;
//     return false;
//    }
        
    // Interleave SVR training, building the model from scratch will all stored data
    SVMParameter param;
    init_svmparam(&param);
    
    SVMNode* x_space;
    x_space = 0;
  
    SVMProblem prob;
    if( !read_problem(tr_data_path.c_str(),x_space,&prob,&param) )
    {
      ROS_ERROR("libsvr read_problem(): failed");
      return false;
    }
    
    SVMModel* model;
    model = svm_train(&prob,&param);

    if( svm_save_model(model_path.c_str(),model) )
    {
      ROS_ERROR("Cannot save svm model.");
      return false;
    }
    
    svm_free_and_destroy_model(&model);
    svm_destroy_param(&param);
    free(prob.y);
    free(prob.x);
    free(x_space);
    
    // Benchmark: obtain prediction error with tr_data 
    // Note that the svm model used to predict is trained/updated using a bunch of data, instead of one-by-one (which wil take long time)
    FILE *input;
    input = fopen(tr_data_path.c_str(),"r");
    if(input == NULL)
    {
      std::cerr << "can't open input file= " << tr_data_path << std::endl;
      return false;
    }
    
    FILE *output;
    output = fopen(fit_data_path.c_str(),"w");// overwrite
    if(output == NULL)
    {
      std::cerr << "can't open output file= " << fit_data_path << std::endl;
      return false;
    }

    SVMModel* trained_model;// trained/updated using a bunch of data from seach for this instance
    if( (trained_model=svm_load_model(model_path.c_str())) == 0 )
    {
      std::cerr << "can't open model file= " << model_path << std::endl;
      return false;
    }

    SVMNode* x;
    int max_nr_attr = 100;
    
    x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));

    int predict_probability=0;
    
    libsvm_predict(trained_model,predict_probability,max_nr_attr,x,input,output);
    
    // TODO make these work, now: causes running-time ERR
//    svm_free_and_destroy_model(&model);
//    free(x);
    fclose(input);
    fclose(output);
    
    // Obtain y_fit from a tmp(hot) file, then put it in ml_data, 
    // have to read fit.out file because libsvm_predict() returns the results in a file
    std::string line;
    
    std::vector<double> fit_out_content;
    std::ifstream fit_out_file(fit_data_path.c_str());
    if(fit_out_file.is_open())
    {
      while ( fit_out_file.good() )
      {
        std::getline(fit_out_file,line);
        if(line.size() != 0)
          fit_out_content.push_back( boost::lexical_cast<double>(line) );
      }
      fit_out_file.close();
    }
    else
    {
     std::cout << "fit_out_file.is_open(): Failed" << std::endl;
     return false;
    }

    std::string tr_data_path_csv;
    tr_data_path_csv = "/home/vektor/hiroken-ros-pkg/learning_machine/data/hot/tr_data.csv";// _must_ be synch with the one in astar_utils.hpp
    
    std::vector<double> y_only_sample_file_content;
    std::ifstream sample_file(tr_data_path_csv.c_str());
    if(sample_file.is_open())
    {
      while ( sample_file.good() )
      {
        std::getline(sample_file,line);
        
        // Parsing the csv, the output is at the last element
        std::vector<std::string> line_parts;
        boost::split( line_parts,line,boost::is_any_of(",") );
        
       if(line.size() != 0)
         y_only_sample_file_content.push_back( boost::lexical_cast<double>(line_parts.back()) );
      }
      sample_file.close();
    }
    else
    {
     std::cout << "sample_file.is_open(): Failed" << std::endl;
     return false;
    }

    if(fit_out_content.size() != y_only_sample_file_content.size())
    {
      ROS_ERROR("fit_out_content.size() != y_only_sample_file_content.size(): mismatch");
      return false;
    }
    else
    {
      n_data_ += y_only_sample_file_content.size();
    }
        
    // Obtain y (true from), then put it in ml_data,
    for(size_t i=0; i < y_only_sample_file_content.size(); ++i)
    {
      // Put the content to variables to make it consistent with the one in LWPR_ONLINE
      std::vector<double> y;
      std::vector<double> y_fit;
      
      y.push_back( fit_out_content.at(i) );
      y_fit.push_back( y_only_sample_file_content.at(i) );
      
      std::vector<double> ml_datum;
      ml_datum.push_back(0.);// 0 TODO think this value
      ml_datum.push_back( y_fit.at(0) );// 1
      ml_datum.push_back( y.at(0) );// 2
      ml_datum.push_back(n_data_);// 3
      
      ml_data.push_back(ml_datum);
    }
  }
  
  if(ml_mode==SVR_OFFLINE or ml_mode==LWPR_ONLINE)
  {
    // Write ml_data
    std::string ml_log_path;
    ml_log_path = std::string(log_path+".ml.log");
    
    std::ofstream ml_log;
    ml_log.open(ml_log_path.c_str(),std::ios::app);// appending because there are multiple instances in a single run
    
    ROS_DEBUG_STREAM("ml_data.size()= " << ml_data.size());
    for(std::vector< std::vector<double> >::const_iterator i=ml_data.begin(); i!=ml_data.end(); ++i)
    {
      ml_log << i->at(0) << "," << i->at(1) << ","<< i->at(2) << "," << i->at(3) << std::endl;
    }

    ml_log.close();
    
    // Write log for heuristics vs true distance 
    // The reference is UCS-sol-path on the rerun base 
    std::string h_log_path;
    h_log_path = std::string(log_path+".h.log");
    
    std::ofstream h_log;
    h_log.open(h_log_path.c_str(),std::ios::app);// appending because there are multiple instances in a single run
    
    TMMVertex curr_vertex;
    curr_vertex = tmm_root_;
    size_t n_hcomp = 0;
    do// Loop for obtaining all paths from any vertex in a solution path to the goal vertex
    {
      double h;// = estimated cost-to-go
      h = get(vertex_heu,tmm_,curr_vertex);
      
      double cost2go = 0.;// valid when cost2go > 0.0
      cost2go = get_cost2go(curr_vertex,tmm_goal_,ucs_sol_tmm);
      
      h_log << h << "," << cost2go << std::endl;
      n_hcomp++;
      
      typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
      tie(oei,oei_end)=out_edges(curr_vertex,ucs_sol_tmm);// no need to iterate over oei because there is only one outedge in the ucs_sol_tmm (it is actually a graph)
     
      curr_vertex = target(*oei,ucs_sol_tmm);
    }
    while(curr_vertex != tmm_goal_);
    
    h_log.close();

    // Store ctamp log for this ctamp instance
    if( !ml_data.empty() )
      ctamp_log->push_back( ml_data.back().at(3) );// at idx=0, note n_samples is casted from size_t to double

    ctamp_log->push_back( n_hcomp ); // at idx=1, note n_samples is casted from size_t to double  
  }

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
//  sol_tmm_dp.property( "weight",get(edge_weight, sol_tmm) );
//  sol_tmm_dp.property( "color",get(edge_color, sol_tmm) );

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
  The tidy config is in a file named "tidy.cfg"
*/
bool
PlannerManager::set_tidy_config()
{
  ObjCfg tidy_cfg;

  std::string  data_path= ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
    
  if( !read_obj_cfg(std::string(data_path+"/tidy.cfg"),&tidy_cfg) )
  {
    ROS_ERROR("Can not find the tidy*.cfg file.");
    return false;
  }
  
  for(ObjCfg::iterator i=tidy_cfg.begin(); i!=tidy_cfg.end(); ++i)
    movable_obj_tidy_cfg_[i->id] = *i; 
  
  return true;
}

//! cost2go()
/*!
  Because we use a solution path from UCS, there must be a path from the start vertex to the goal vertex.
  It is a graph _not_ a multigraph
  All node but the goal node have exactly one out edge
*/
double
PlannerManager::get_cost2go(const TMMVertex& start,const TMMVertex& goal,const TaskMotionMultigraph& tmm)
{
  double cost2go = 0.;
  
  TMMVertex v;
  v = start;
  do
  {
    typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
    tie(oei,oei_end) = out_edges(v,tmm);
    
    double cost;
    cost = get(edge_weight,tmm,*oei);
//    ROS_DEBUG_STREAM("get(edge_weight,tmm,*oei)= " << get(edge_weight,tmm,*oei));
    
    cost2go += cost;
    
    v = target(*oei,tmm);
//    ROS_DEBUG_STREAM("get(vertex_name,tmm_,v)=" << get(vertex_name,tmm_,v));
  }
  while(v != goal);
    
  return cost2go;
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
