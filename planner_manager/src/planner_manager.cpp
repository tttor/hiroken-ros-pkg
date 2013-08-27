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
  
  n_ctamp_attempt_ = 0;
  n_ml_update_ = 0;
  n_svr_training_data_ = 0;
  n_ml_update_ = 0;
  
  lwpr_model_ = new LWPR_Object(ml_util::LWPR_INPUT_DIM,ml_util::LWPR_OUTPUT_DIM);
}
//! A destructor.
/*!
  The destructor does nothing.
*/
PlannerManager::~PlannerManager()
{ 
  delete lwpr_model_;
}

//! collision_object_cb
/*!
  Movable object id -> CAN1, CAN2. ..., CANn
  Unmovable object id -> unmovable.table, unmovable.vase, ...
*/
void 
PlannerManager::collision_object_cb(const arm_navigation_msgs::CollisionObject::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Heard= " << msg->id.c_str());
  
  // Distinguish unmovable object
  std::vector<std::string> id_parts;// e.g. "unmovable.vase", "CAN1"
  boost::split( id_parts,msg->id,boost::is_any_of(".") );
  
  std::string type;
  type = id_parts.at(0);

  std::map<std::string, arm_navigation_msgs::CollisionObject>::iterator it;
  bool inserted;
    
  if( !strcmp(type.c_str(),"unmovable") )
  {
    boost::tie(it,inserted) = unmovable_obj_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );
  }
  else// movable ones
  {
    boost::tie(it,inserted) = movable_obj_messy_cfg_.insert( std::pair<std::string, arm_navigation_msgs::CollisionObject>(msg->id, *msg) );
  }
  
  if(!inserted)
    it->second = *msg;// Update with the newer one
}

bool
PlannerManager::plan_srv_handle(planner_manager::Plan::Request& req, planner_manager::Plan::Response& res)
{
  return plan( req.ml_mode,req.rerun,req.ml_hot_path,req.log_path,&(res.ctamp_sol),&(res.ctamp_log) );
}

bool
PlannerManager::clear_n_ctamp_attempt_srv_handle(planner_manager::Misc::Request& req, planner_manager::Misc::Response& res)
{
  n_ctamp_attempt_ = 0;
  
  return true;
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
PlannerManager::plan(const size_t& ml_mode,const bool& rerun,const std::string& ml_hot_path,const std::string& log_path,std::vector<trajectory_msgs::JointTrajectory>* ctamp_sol,std::vector<double>* ctamp_log)
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
  tmm_ = TaskMotionMultigraph();// renew the tmm_, necessary because there are multiple attempts in an episode
  ucs_tmm_ = TaskMotionMultigraph();// renew the ucs_tmm_, necessary because there are multiple attempts in an episode
    
  if(!rerun)
  {
    // Create task plan space encoded in a task motion graph
    // A call to the task planner if succeed outputs a file: vanilla_tmm.dot
    SymbolicPlannerManager spm(this);
    
    if( !spm.plan() )
    {
      cerr << "from spm.plan()" << endl;
      ROS_ERROR("Can not construct TMM!");
      return false;
    }
    ROS_DEBUG("TMM is constructed successfully");
  }
  
  // Read the vanilla tmm
  boost::dynamic_properties tmm_dp;
  
  tmm_dp.property("vertex_id", get(vertex_name, tmm_));
  
  tmm_dp.property( "label", get(edge_name, tmm_) );
  tmm_dp.property( "weight", get(edge_weight, tmm_) );
  tmm_dp.property( "jspace", get(edge_jspace, tmm_) );
  tmm_dp.property( "color",get(edge_color, tmm_) );
  tmm_dp.property( "srcstate",get(edge_srcstate, tmm_) );  
  tmm_dp.property( "mptime",get(edge_mptime, tmm_) );  
  tmm_dp.property( "planstr",get(edge_planstr, tmm_) );
  
  std::string tmm_dot_path = data_path + "/vanilla_tmm.dot";  
  std::ifstream tmm_dot(tmm_dot_path.c_str());
  
  ROS_DEBUG_STREAM("Going to read " << tmm_dot_path);
  if( !read_graphviz(tmm_dot, tmm_, tmm_dp, "vertex_id") )
  {
    ROS_ERROR("read_graphviz(vanilla_tmm.dot): Failed");
    return false;
  }
  ROS_DEBUG("read_graphviz(vanilla_tmm.dot): Succeeded");
  
  // Need to (re)-initialize the edge color to "black" as in the past, in the vanilla_tmm, they are initialized with empty strings
  graph_traits<TaskMotionMultigraph>::edge_iterator ei,ei_end;
  for(tie(ei,ei_end)=edges(tmm_); ei!=ei_end; ++ei)
  {
    put( edge_color,tmm_,*ei,std::string("black") );
    put( edge_weight,tmm_,*ei,std::numeric_limits<double>::max() );
  }

  // Retrieve the UCS-planned TMM only if rerun (=benchmarked attempt)
  // Assume that the base directory for rerun contains UCS-planned TMM having a CTAMP solution  
  if(rerun)
  {
    boost::dynamic_properties ucs_tmm_dp;
    
    ucs_tmm_dp.property("vertex_id", get(vertex_name, ucs_tmm_));
    
    ucs_tmm_dp.property( "label",get(edge_name, ucs_tmm_) );
    ucs_tmm_dp.property( "weight",get(edge_weight, ucs_tmm_) );  
    ucs_tmm_dp.property( "jspace",get(edge_jspace, ucs_tmm_) );
    ucs_tmm_dp.property( "color",get(edge_color, ucs_tmm_) );
    ucs_tmm_dp.property( "srcstate",get(edge_srcstate, ucs_tmm_) );  
    ucs_tmm_dp.property( "mptime",get(edge_mptime, ucs_tmm_) );  
    ucs_tmm_dp.property( "planstr",get(edge_planstr, ucs_tmm_) );
    // Notice: the edge_plan property is not initialized.

    std::ifstream ucs_tmm_dot(  std::string(base_data_path+"/tmm.dot").c_str()  );
    if( !read_graphviz(ucs_tmm_dot, ucs_tmm_, ucs_tmm_dp, "vertex_id") )
    {
      ROS_ERROR("read_graphviz(ucs_tmm_dot,...): Failed.");
      return false;
    }
    ROS_DEBUG("read_graphviz(ucs_tmm_dot,...): Succeeded");
  }// if(rerun)
  
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
  
  // perf-log related var
  // The "perf.log.csv" is a single liner text file containing 
  // (1)CTAMP_SearchTime,(2)total_mp_time,(3)#ExpandedVertices,(4)#Vertices,(5)SolPathCost,(6)#Vertices in solution path,(7)#exp_op,(8)Search sol. cost,(9)Search sol. #vertices, DO NOT forget to resize the vector perf_log below!
  std::vector<double> perf_log;
  perf_log.resize(9);
  
  std::ofstream perf_log_out;
  perf_log_out.open(std::string(data_path+"/perf.log").c_str());

  // ml-related var
  std::vector< std::vector<double> > ml_data;
  std::string ml_pkg_path = ros::package::getPath("learning_machine");
  
  // Search 
  GeometricPlannerManager gpm(this);
  std::vector<TMMVertex> predecessors(num_vertices(tmm_));
  std::vector<double> distances(num_vertices(tmm_));
  
  double search_time = 0.;// initially is gross, but then becomes net
  double total_gp_time = 0.;// recorded by the visitor during search (may be zero if in rerun modes), used to substract the gross search time
  double total_mp_time = 0.;// recorded by the visitor during search (may be zero if in rerun modes), used to substract the gross search time,
  
  size_t n_exp_op = 0;// the number of vertex expansion operations
  size_t n_exp_vert = 0;// the number of unique vertex expansion

  std::vector<TMMEdge> sol_path;// i.e. the $p_s$

  ros::Time search_begin = ros::Time::now();
  try 
  {
    switch(ml_mode)
    {
      case ml_util::NO_ML:
      {
        // use "as is" mode= ml_util::SVR_OFFLINE, 
        // the learner is useless though, see astar_utils.hpp where the heuristic for this mode is always set to h= 0
        ROS_DEBUG("learner= ml_util::NO_ML but via learner= SVR");
      }
      case ml_util::NO_ML_BUT_COLLECTING_SAMPLES:
      {
        // use "as is" mode= ml_util::SVR_OFFLINE, 
        // the learner is useless though, see astar_utils.hpp where the heuristic for this mode is always set to h= 0
        ROS_DEBUG("learner= ml_util::NO_ML_BUT_COLLECTING_SAMPLES but via learner= SVR");
      }
      case ml_util::SVR_OFFLINE:
      {
        ROS_DEBUG("learner= SVR");
        
        size_t search_heuristic_ml_mode;
        if((ml_mode == ml_util::SVR_OFFLINE) and (n_ctamp_attempt_==0) )
          search_heuristic_ml_mode = ml_util::NO_ML;// for the first ctamp instance, use ml_util::NO_ML because there is no trained-SVR yet.
        else
          search_heuristic_ml_mode = ml_mode;// 3 possible values of ml_mode (as we use two cases above w/o breaks): NO_ML and SVR_OFFLINE and NO_ML_BUT_COLLECTING_SAMPLES
        
        // SVR from libsvm
        SVM_Object learner(svr_model_,ml_util::SVR_MAX_N_ATTR);

        ROS_DEBUG("Searching over TMM ...");
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,SVM_Object>(tmm_goal_,&gpm,&learner,search_heuristic_ml_mode,prep_data_,&n_ml_update_)
                    , visitor( AstarVisitor<TaskMotionMultigraph,SVM_Object>(tmm_goal_,&gpm,&learner,&ml_data,ml_mode,ml_hot_path,&total_gp_time,&total_mp_time,&n_exp_op,&n_ml_update_) )
                    . predecessor_map(&predecessors[0])
                    . distance_map(&distances[0])
                    );
        
        break;
      }
      case ml_util::LWPR_ONLINE:
      {
        ROS_DEBUG("learner= LWPR");// LWPR from Edinburg Univ.
        
        if(n_ctamp_attempt_ == 0)
        {
          // Initialize the vanilla LWPR model
          lwpr_model_->useMeta(true);// Determines whether 2nd order distance matrix updates are to be performed
          lwpr_model_->updateD(ml_util::TUNED_LWPR_UPDATE_D);// Determines whether distance matrix updates are to be performed            
          lwpr_model_->setInitD(ml_util::TUNED_LWPR_D);/* Set initial distance metric to D*(identity matrix) */
          lwpr_model_->setInitAlpha(ml_util::TUNED_LWPR_ALPHA);/* Set init_alpha to _alpha_ in all elements */
          lwpr_model_->penalty(ml_util::TUNED_LWPR_PEN);
          
          ROS_DEBUG("lwpr_model_: INITIALIZED");
        }
        
        ROS_DEBUG("Searching over TMM ...");
        astar_search( tmm_
                    , tmm_root_
                    , AstarHeuristics<TaskMotionMultigraph,double,LWPR_Object>(tmm_goal_,&gpm,lwpr_model_,ml_mode,prep_data_,&n_ml_update_)
                    , visitor( AstarVisitor<TaskMotionMultigraph,LWPR_Object>(tmm_goal_,&gpm,lwpr_model_,&ml_data,ml_mode,ml_hot_path,&total_gp_time,&total_mp_time,&n_exp_op,&n_ml_update_) )
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
    
    // Get elapsed time for planning that is represented by the search process, 
    // but _excluding_ grasp planning time (because we do not care it for now) and motion planning time (because it is stored separately in edge_mptime)
    search_time = (ros::Time::now()-search_begin).toSec();
    search_time -= total_gp_time;
    search_time -= total_mp_time;
    
    // Get #expanded vertices + total_mp_time
    // The total_mp_time is the sum of (so that TOTAL) motion planning time for each out-edges.
    // All time used by other processes in a vertex expansion is included in the search_time (which holds the overall time for search)
    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
    for(boost::tie(vi,vi_end) = vertices(tmm_); vi!=vi_end; ++vi)
    {
      if(get(vertex_color,tmm_,*vi) == color_traits<boost::default_color_type>::black())
      {
        ++n_exp_vert;
      }
    }
        
    // Convert from vector to list to enable push_front()
    std::list<TMMVertex> sol_path_vertex_list;
    for(TMMVertex v = tmm_goal_; ; v = predecessors[v]) 
    {
      sol_path_vertex_list.push_front(v);
      if(predecessors[v] == v)
        break;
    }
    
    // Build the solution path $p_s$
    std::list<TMMVertex>::iterator spvl_it = sol_path_vertex_list.begin();

    TMMVertex s;
    s = *spvl_it;
    
    for(++spvl_it; spvl_it != sol_path_vertex_list.end(); ++spvl_it)
    {
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
  }// End of: catch(FoundGoalSignal fgs) 
  
  // The confirmation step
  std::vector<TMMEdge> copy_sol_path = sol_path;// the copy is for debugging purpose
  
  for(std::vector<TMMEdge>::iterator i=sol_path.begin(); i!=sol_path.end(); ++i)
  {
    if( get(edge_planstr,tmm_,*i).size() == 0 )// Use edge_planstr, instead of edge_plan since in rerun modes, the latter does not hold any plan
    {
      sol_path.clear();
      ctamp_sol->clear();
      
      break;
    }
    else
    {
      put( edge_color,tmm_,*i,std::string("blue") );
      put( vertex_color,tmm_,source(*i,tmm_),color_traits<boost::default_color_type>::gray() );// for vertex in the solution path
      put( vertex_color,tmm_,target(*i,tmm_),color_traits<boost::default_color_type>::gray() );// somewhat redundant indeed
      
      ctamp_sol->push_back( get(edge_plan,tmm_,*i) );
    }
  }
  
  // Log the CTAMP process: search + confirmation
  cout << "===CTAMP process log===" << endl;
  
  cout << "CTAMP_SearchTime= " << search_time << endl;
  perf_log_out << "CTAMP_SearchTime=" << search_time << endl;
  perf_log.at(0) = search_time;
  
  cout << "total_mp_time= " << total_mp_time << endl;
  perf_log_out << "total_mp_time=" << total_mp_time << endl;
  perf_log.at(1) = total_mp_time;
  
  cout << "#ExpandedVertices= " << n_exp_vert << endl;
  perf_log_out << "#ExpandedVertices=" << n_exp_vert << endl;
  perf_log.at(2) = n_exp_vert;
  
  cout << "#Vertices= " << num_vertices(tmm_) << endl;
  perf_log_out << "#Vertices=" << num_vertices(tmm_) << endl;
  perf_log.at(3) = num_vertices(tmm_);
  
  cout << "SolutionPath(e)=" << endl;
  perf_log_out << "SolutionPath(e)=";
  for(std::vector<TMMEdge>::iterator i=copy_sol_path.begin(); i!=copy_sol_path.end(); ++i)// NOTE: Using copy_sol_path, instead of sol_path
  {
    cout << get(edge_name,tmm_,*i) << "[" << get(edge_jspace,tmm_,*i) << "]" << " --> " << get(edge_color,tmm_,*i) << endl;
    perf_log_out << get(edge_name,tmm_,*i) << "[" << get(edge_jspace,tmm_,*i) << "]" << ",";
  }
  perf_log_out << endl;

  perf_log.at(6) = n_exp_op;
  perf_log.at(7) = distances[tmm_goal_];
  perf_log.at(8) = sol_path.size() + 1;

  if( sol_path.empty() )// did not pass the confirmation step
  {
    cout << "SolPathCost= " << distances[tmm_goal_] << "(but zeroed because not passed)" << endl;
    perf_log_out << "SolPathCost= " << distances[tmm_goal_] << "(but zeroed because not passed)" << endl;
    perf_log.at(4) = 0.;// an invalid value, cost > 0
    perf_log.at(5) = 0.;// an invalid value, |sol| > 0  
  }
  else// passed the confirmation step
  {
    cout << "SolPathCost= " << distances[tmm_goal_] << endl;
    perf_log_out << "SolPathCost=" << distances[tmm_goal_] << endl;
    perf_log.at(4) = distances[tmm_goal_];
    perf_log.at(5) = sol_path.size() + 1;
  }
          
  utils::write_csv(perf_log,std::string(data_path+"/perf.log.csv"));
  
  // Do interleaved training
  if(ml_mode==ml_util::SVR_OFFLINE)
  {
    // Initialize
    std::string raw_tr_data_path = std::string(ml_hot_path+"/tr_data.csv");// raw means un-preprocessed, produced by search examine_vertex() visitor
    std::string preprocessed_tr_data_path = std::string(ml_hot_path+"/preprocessed_tr_data.csv");// contains pca-preprocessed version of tr_data.csv
    std::string preprocessed_tr_data_path_2 = std::string(ml_hot_path+"/preprocessed_tr_data.libsvmdata");// contains pca-preprocessed data in libsvmdata format, which is converted from pca_tr_data.csv
        
    std::string raw_delta_tr_data_path = std::string(ml_hot_path+"/delta_tr_data.csv");// delta samples are used to calculate testing errors
    
    std::string tmp_data_path  = std::string(ml_hot_path+"/fit.out");// _must_ be always overwritten, contains fitting values of the SVR
    
    // Obtain testing err, iff the model is already trained/updated, at least, once  
    // Note that the testing data must be preprocessed with the same params used to trained the model
    std::vector<double> y_te_true;
    std::vector<double> y_te_fit;
    
    if(n_ml_update_ > 0)
    {
      SVMModel* old_svr_model;
      old_svr_model = svr_model_;
      
      // Obtain y_te_true from delta_tr_data
      y_te_true =  ml_util::get_y_true(raw_delta_tr_data_path);
      
      // Obtain input samples X_te from delta_tr_data
      std::vector<Input> X;
      X = ml_util::get_X(raw_delta_tr_data_path);
      
      // Preprocess input samples with the curret prep_data_ params, like used in computing the heuristics
      std::vector<Input> preprocessed_X;
      ml_util::preprocess(X,prep_data_,&preprocessed_X);
      
      // Get y_te_fit
      for(size_t i=0; i < preprocessed_X.size(); ++i)
      {
        Input in;
        in = preprocessed_X.at(i);
        
        // Construct input x: convert Input& to SVMNode*
        SVMNode* x;
        x = (struct svm_node *) malloc(ml_util::SVR_MAX_N_ATTR*sizeof(struct svm_node));

        size_t idx = 0 ;
        for(Input::const_iterator j=in.begin(); j!=in.end(); ++j)
        {
          x[idx].index = (int)(j-in.begin()) + 1;// libsvm manual: <index> is an integer starting from 1 and _must_ be in Ascending order
          x[idx].value = *j;
          
          ++idx;
        }
        x[idx].index = -1;// index = -1 indicates the end of one vector

        // Predict
        y_te_fit.push_back( svm_predict(old_svr_model,x) );
      }
    }//if(n_ml_update_ > 0)
    
    if(y_te_true.size() != y_te_fit.size())
    {
      cerr << "y_te_true.size() != y_te_fit.size() -> both are cleared" << endl;
      y_te_true.clear();
      y_te_fit.clear();
    }
    
    // Pre-process the training data: PCA
    prep_data_.dim_red = ml_util::TUNED_SVR_DIM_RED;
    prep_data_.lo_dim = ml_util::TUNED_SVR_LO_DIM;
    if( !ml_util::preprocess_data(raw_tr_data_path,preprocessed_tr_data_path,&prep_data_) )
    {
      ROS_ERROR("ml_util::preprocess_data() FAILED");
      return false;
    }

    if( !data_util::convert_csv2libsvmdata(preprocessed_tr_data_path,preprocessed_tr_data_path_2) )//overwritten here!
    {
      ROS_ERROR("data_util::convert_csv2libsvmdata() FAILED");
      return false;
    }

    std::vector<double> y_true;
    y_true = ml_util::get_y_true_libsvmdata(preprocessed_tr_data_path_2);
    
    // Sanity checks
    if( (utils::get_n_lines(raw_tr_data_path) != utils::get_n_lines(preprocessed_tr_data_path)) 
        or (utils::get_n_lines(raw_tr_data_path) != utils::get_n_lines(preprocessed_tr_data_path_2)) 
        or (utils::get_n_lines(raw_tr_data_path) != y_true.size()) )
    {
      cerr << "utils::get_n_lines(raw_tr_data_path)= " << utils::get_n_lines(raw_tr_data_path) << endl;
      cerr << "utils::get_n_lines(preprocessed_tr_data_path)= " << utils::get_n_lines(preprocessed_tr_data_path) << endl;
      cerr << "utils::get_n_lines(preprocessed_tr_data_path_2)= " << utils::get_n_lines(preprocessed_tr_data_path_2) << endl;
      cerr << "y_true.size()= " << y_true.size() << endl;
      
      return false;
    }
    
    // Interleave SVR training, building the model from scratch will all stored data in tr_data.csv
    ROS_DEBUG("SVR training ...");
    
    SVMParameter param;
    init_svmparam(&param);
    param.C = ml_util::TUNED_SVR_C;
    param.p = ml_util::TUNED_SVR_P;
    param.kernel_type = ml_util::TUNED_SVR_KERNEL_TYPE;
    param.gamma = ml_util::TUNED_SVR_GAMMA;

    SVMNode* x_space;
    x_space = 0;
  
    SVMProblem problem;
    if( !read_problem(preprocessed_tr_data_path_2.c_str(),x_space,&problem,&param) )
    {
      ROS_ERROR("libsvr read_problem(): failed");
      return false;
    }
    
    svr_model_ = svm_train(&problem,&param);
    ++n_ml_update_;
    n_svr_training_data_ = y_true.size();// assignment, not increment as we use all available tr samples
    
//    if( svm_save_model(model_path.c_str(),svr_model_) )
//    {
//      ROS_ERROR("Cannot save svm model.");
//      return false;
//    }
    svm_destroy_param(&param); free(problem.y); free(problem.x); free(x_space);
    
    // Benchmark: obtain prediction error with tr_data + put data into ml_data
    // Note that the svm model used to predict is trained/updated using a bunch of data (samples obtained from this CTAMP instance), instead of being trained one-by-one (which wil take long time)
    // Note that tr_mse is obtained from all available samples so far (the same as what used in updating the ML model), insted of only delta samples
    ROS_DEBUG("Obtaining tr_err...");
    
    FILE *input;
    FILE *output;
    SVMNode* x = (struct svm_node *) malloc( ml_util::SVR_MAX_N_ATTR*sizeof(struct svm_node));// for inputs 
    
    input = fopen(preprocessed_tr_data_path_2.c_str(),"r");// Use all tr_data, same as what used to train the SVR before
    if(input == NULL)
    {
      std::cerr << "can't open input file= " << preprocessed_tr_data_path_2 << std::endl;
      return false;
    }
    
    output = fopen(tmp_data_path.c_str(),"w");// overwrite
    if(output == NULL)
    {
      std::cerr << "can't open output file= " << tmp_data_path << std::endl;
      return false;
    }
    
    double MSE;
    MSE = libsvm_predict(svr_model_,0,ml_util::SVR_MAX_N_ATTR,x,input,output);// 2nd arg: predict_probability=0
    fclose(input); fclose(output); free(x); 
    cerr << "utils::get_n_lines(tmp_data_path) y_tr_fit.size = " << utils::get_n_lines(tmp_data_path) << endl;
    
    // Calculate nMSE, which is an MSE normalized by the variance of true output
    double var;
    var = ml_util::get_var(y_true);
    
    double nMSE;
    nMSE = MSE/var;
    
    // Put the content to ml_data variable, to be as similiar as possible with the one in ml_util::LWPR_ONLINE
    // The number of data follow the size of y_te_true if it is not empty, otherwise n data = 1
    std::vector<double> ml_datum;
    ml_datum.resize(6);// Elements: see below; yes, all but testing data are duplicated
    
    size_t n_raw_svr_training_data;// must be equal to n_svr_training_data_ because preprocess does not remove duplicates
    n_raw_svr_training_data = utils::get_n_lines(raw_tr_data_path);
      
    if( y_te_true.empty() )
    {
      ml_datum.at(0) = -1.0;// an invalid value for y_te_fit
      ml_datum.at(1) = nMSE;
      ml_datum.at(2) = -1.0;// an invalid value for y_te_true
      ml_datum.at(3) = n_svr_training_data_;
      ml_datum.at(4) = MSE;
      ml_datum.at(5) = n_raw_svr_training_data;
      
      ml_data.push_back(ml_datum);
    }
    else
    {
      for(size_t i=0; i < y_te_true.size(); ++i)
      {
        ml_datum.at(0) = y_te_fit.at(i);
        ml_datum.at(1) = nMSE;
        ml_datum.at(2) = y_te_true.at(i);
        ml_datum.at(3) = n_svr_training_data_;
        ml_datum.at(4) = MSE;
        ml_datum.at(5) = n_raw_svr_training_data;
        
        ml_data.push_back(ml_datum);
      }
    }
        
    // Remove deltas after every te_err retrieval; clear after one ctamp attempt
    // Plus some supporting files
    boost::filesystem::remove( boost::filesystem::path(raw_delta_tr_data_path) );
    boost::filesystem::remove( boost::filesystem::path(preprocessed_tr_data_path) );
    boost::filesystem::remove( boost::filesystem::path(preprocessed_tr_data_path_2) );
  }// if(ml_mode==ml_util::SVR_OFFLINE)
  
  // Logging of ML-related data and heuristic-related data
  if(ml_mode==ml_util::SVR_OFFLINE or ml_mode==ml_util::LWPR_ONLINE)
  {
    // Write ml_data
    std::string ml_log_path;
    ml_log_path = std::string(log_path+".ml.log");
    
    std::ofstream ml_log;
    ml_log.open(ml_log_path.c_str(),std::ios::app);// appending because there are multiple instances in a episode
    
    for(std::vector< std::vector<double> >::const_iterator i=ml_data.begin(); i!=ml_data.end(); ++i)
    {
      for(size_t j=0; j < i->size(); ++j)
        ml_log << i->at(j) << ",";

      ml_log << n_ctamp_attempt_+1 << std::endl;// n-th attempt when this data comes from, plus 1 because n_ctamp_attempt_ is incremented by one later at the end
    }
    ml_log.close();
    
    // Write h_log for heuristics vs true distance 
    // The true_cost-to-go is obtained from UCS-planned TMM using Djikstra (filtered for planned_edge_only)for every est-cost-to-go in the current Astar-planned tmm
    std::string h_log_path;
    h_log_path = std::string(log_path+".h.log");
    
    std::ofstream h_log;
    h_log.open(h_log_path.c_str(),std::ios::app);// appending because there are multiple instances in a episode

    std::set<TMMVertex> expvert_plus_set;// expanded vertices plus vertices in the open-list/frontier, they are vertices that have heuristics
     
    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
    for(boost::tie(vi,vi_end) = vertices(tmm_); vi!=vi_end; ++vi)
    {
      if( (get(vertex_color,tmm_,*vi)==color_traits<boost::default_color_type>::black())// expanded
          or
          (get(vertex_color,tmm_,*vi)==color_traits<boost::default_color_type>::gray())// in the solution path, which must be expanded
        )
      {
        expvert_plus_set.insert(*vi);
        
        // Do this because we do not have any flag for vertex that is ever in the open-list/frontier
        graph_traits<TaskMotionMultigraph>::adjacency_iterator avi, avi_end;
        for(tie(avi,avi_end)=adjacent_vertices(*vi,tmm_); avi!=avi_end; ++avi )  
        {
          expvert_plus_set.insert(*avi);
        }
      }
    }
    
    PlannedEdgeFilter<TMMEdgeColorMap> planned_edge_filter( get(edge_color, tmm_) );
    typedef filtered_graph< TaskMotionMultigraph, PlannedEdgeFilter<TMMEdgeColorMap> > UCSPlannedEdgeOnlyTMM;
    UCSPlannedEdgeOnlyTMM filtered_ucs_tmm(ucs_tmm_,planned_edge_filter);// for now, we do not struggle doing planning for un-planned edge, instead we assume that those edges "truly, as the true value" have very high cost to the goal
    
    size_t n_hcmp = 0;// the number of (est-cost2go vs. true-cost2go) obtained from this attempt
    for(std::set<TMMVertex>::iterator i=expvert_plus_set.begin(); i!=expvert_plus_set.end(); ++i)
    {
      if(*i == tmm_goal_)
        continue;// we do not consider h(v=goal)
      
      double h;// = estimated cost-to-go
      h = get(vertex_heu,tmm_,*i);
      
      // Obtain true_cost-to-go using the Djikstra
      double cost2go = 0.;// initialize with an invalid cost 
            
      std::vector< graph_traits<UCSPlannedEdgeOnlyTMM>::vertex_descriptor > parents(num_vertices(filtered_ucs_tmm));
      typedef graph_traits<UCSPlannedEdgeOnlyTMM>::vertices_size_type size_type;
      for(size_type p=0; p < num_vertices(filtered_ucs_tmm); ++p)
        parents.at(p) = p;
      
      graph_traits<UCSPlannedEdgeOnlyTMM>::vertex_descriptor dijkstra_src;
      dijkstra_src = *i;
      
      dijkstra_shortest_paths( filtered_ucs_tmm,dijkstra_src,predecessor_map(&parents[0]) );
      
      if(parents.at(tmm_goal_) != tmm_goal_)// then, there is a path in ucs-planned TMM from src to the goal
      {
        // Backtrack to get the path cost2go from src to the goal vertex
        graph_traits<UCSPlannedEdgeOnlyTMM>::vertex_descriptor hot_v = tmm_goal_;
        graph_traits<UCSPlannedEdgeOnlyTMM>::vertex_descriptor child = hot_v;
        
        do
        {
          double cheapest_cost = std::numeric_limits<double>::max();// of the several edges of (parent of hot_v)-->(hot_v), can not use boost::edge() due to multigraph
          graph_traits<UCSPlannedEdgeOnlyTMM>::out_edge_iterator oei,oei_end;
          for(boost::tie(oei,oei_end) = out_edges(parents.at(hot_v),filtered_ucs_tmm); oei!=oei_end; ++oei)
          {
            if(target(*oei,filtered_ucs_tmm) == hot_v)
            {
              double cost;
              cost = get(edge_weight,filtered_ucs_tmm,*oei);
              
              if(cost < cheapest_cost) 
                cheapest_cost = cost;
            }
          }
          cost2go += cheapest_cost;

          child = hot_v;
          hot_v = parents.at(hot_v);
        }
        while(hot_v != parents.at(hot_v));
      }
      else
      {
        // Assume that if no path from this vertex to the goal on planned_edge_only ucs tmm then the true cost must be very high
        cost2go += std::numeric_limits<double>::max();
      }
      
      // Write
      h_log << h 
            << "," << cost2go 
            << ","<< n_ctamp_attempt_+1 // n-th attempt when this data comes from, plus 1 because n_ctamp_attempt_ is incremented by one later at the end
            /* << "," << get(vertex_name,filtered_ucs_tmm,*i) */ 
            << std::endl;
      ++n_hcmp;
    }// for each vertex in expvert_plus_set
    
    h_log.close();

    // Write h2_log for checking heuristic consistency: h(v) <= c(v,v') + h (v')
    std::string h2_log_path;
    h2_log_path = std::string(log_path+".h2.log");
    
    std::ofstream h2_log;
    h2_log.open(h2_log_path.c_str(),std::ios::app);// appending because there are multiple instances in a episode
    
    size_t n_hcmp2 = 0;// the number of (h_v, h_av, c_v_av)
        
//    boost::graph_traits<TaskMotionMultigraph>::vertex_iterator vi, vi_end;
    for(boost::tie(vi,vi_end) = vertices(tmm_); vi!=vi_end; ++vi)
    {
      if( (get(vertex_color,tmm_,*vi)==color_traits<boost::default_color_type>::black())// expanded
          or
          (get(vertex_color,tmm_,*vi)==color_traits<boost::default_color_type>::gray())// in the solution path, which must be expanded
        )
      {
        double h_v;
        h_v = get(vertex_heu,tmm_,*vi);
  
        std::map<TMMVertex,double> av_cost_map;// Use this instead of simply iterating over adjacent_vertices because this is _multigraph_
        typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei,oei_end;
        for(tie(oei,oei_end)=out_edges(*vi,tmm_);oei != oei_end; ++oei)
        {
          TMMVertex av;
          av = target(*oei,tmm_);
          
          double c;
          c = get(edge_weight,tmm_,*oei);
          
          std::map<TMMVertex,double>::iterator it;
          bool inserted;
          boost::tie(it,inserted) = av_cost_map.insert( std::make_pair(av,c) );
          
          // Put the lowest cost
          if( (!inserted)and(c < it->second) )
            it->second = c;
        }
        
        for(std::map<TMMVertex,double>::iterator i = av_cost_map.begin(); i != av_cost_map.end(); ++i)
        {
          double h_av;
          h_av = get(vertex_heu,tmm_,i->first);
          
          double c_v_av;
          c_v_av = i->second;
          
          h2_log << h_v 
                 << "," << h_av 
                 << "," << c_v_av 
                 << ","<< n_ctamp_attempt_+1 // n-th attempt when this data comes from, plus 1 because n_ctamp_attempt_ is incremented by one later at the end
                 /*<< "," << get(vertex_name,tmm_,*vi) << "," << get(vertex_name,tmm_,i->first) */ 
                 << std::endl;
          ++n_hcmp2;
        }
      }
    }
    
    h2_log.close();
    
    // Store ctamp log for this ctamp instance
    size_t n_samples;
    if( !ml_data.empty() ) 
      n_samples = ml_data.back().at(3);// at idx=0, note n_samples is casted from size_t to double
    else
      n_samples = 0;
    
    ctamp_log->resize(3);// Elements: n_samples and n_hcmp and n_hcmp2
    ctamp_log->at(0) = n_samples;
    ctamp_log->at(1) = n_hcmp;
    ctamp_log->at(2) = n_hcmp2;
  }// if(ml_mode==ml_util::SVR_OFFLINE or ml_mode==ml_util::LWPR_ONLINE)

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
  sol_tmm_dp.property( "weight",get(edge_weight, sol_tmm) );  
  sol_tmm_dp.property( "jspace",get(edge_jspace, sol_tmm) );
  sol_tmm_dp.property( "color",get(edge_color, sol_tmm) );
  sol_tmm_dp.property( "srcstate",get(edge_srcstate, sol_tmm) );  
  sol_tmm_dp.property( "mptime",get(edge_mptime, sol_tmm) );  
  sol_tmm_dp.property( "planstr",get(edge_planstr, sol_tmm) );

  std::string sol_tmm_dot_path = data_path + "/sol_tmm.dot";
  ofstream sol_tmm_dot;
  sol_tmm_dot.open(sol_tmm_dot_path.c_str());
    
  write_graphviz_dp( sol_tmm_dot, sol_tmm, sol_tmm_dp, std::string("vertex_id"));
  sol_tmm_dot.close();
  
  // Addition info. for perf.log
  perf_log_out << "tidy.cfg=" << base_data_path << tidy_cfg_filename;
  
  // Closure
  perf_log_out.close();
  ++n_ctamp_attempt_;
  return true;
}

//! Set the tidy_config_
/*!
  The tidy config is in a file named "tidy.cfg"
*/
bool
PlannerManager::set_tidy_config(bool random)
{
  utils::ObjCfg tidy_cfg;

  std::string  data_path= ".";
  if( !ros::param::get("/data_path", data_path) )
    ROS_WARN("Can not get /data_path, use the default value instead");
  
  if(random)// WARN: Only one object for now
  {
    arm_navigation_msgs::CollisionObject object;
    arm_navigation_msgs::Shape object_shape;
    geometry_msgs::Pose object_pose;
    
    object.id = "CAN1";
    object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    object.header.frame_id = "/table";
    
    object_pose.position.z = (utils::TABLE_THICKNESS/2)+(utils::B_HEIGHT/2);    
    
    // Randomize positions 
//    // for CLUSTER.1, comment another
//    boost::normal_distribution<> normal_dist_x(0.000,0.070);
//    boost::normal_distribution<> normal_dist_y(0.420,0.070);
    // for CLUSTER.2, comment another
    boost::normal_distribution<> normal_dist_x(-0.420,0.070);
    boost::normal_distribution<> normal_dist_y(0.000,0.070);

    boost::variate_generator<utils::RandomNumberGenerator&, boost::normal_distribution<> > x_gen(utils::g_rng,normal_dist_x);    
    boost::variate_generator<utils::RandomNumberGenerator&, boost::normal_distribution<> > y_gen(utils::g_rng,normal_dist_y);
    double x, y;
    while( true and ros::ok() )
    {
      x = x_gen();
      y = y_gen();
      
      double r_here;
      r_here = sqrt( pow(x,2)+pow(y,2) );
      
      if(r_here < (utils::TABLE_RADIUS-utils::B_HEIGHT))
        break;
    }

    object_pose.position.x = x;
    object_pose.position.y = y;

    object_pose.orientation.x = 0.000;
    object_pose.orientation.y = 0.000;
    object_pose.orientation.z = 0.000;
    object_pose.orientation.w = 1.000;
    
    object_shape.type = arm_navigation_msgs::Shape::CYLINDER;// TODO make it flexible if necessary
    object_shape.dimensions.resize(2);
    object_shape.dimensions[0] = utils::B_RADIUS;
    object_shape.dimensions[1] = utils::B_HEIGHT;
    
    object.shapes.push_back(object_shape);
    object.poses.push_back(object_pose);

    tidy_cfg.push_back(object);
  }
  else
  {
    if( !utils::read_obj_cfg(std::string(data_path+"/tidy.cfg"),&tidy_cfg) )
    {
      ROS_ERROR("Can not find the tidy*.cfg file.");
      return false;
    }
  }
  
  for(utils::ObjCfg::iterator i=tidy_cfg.begin(); i!=tidy_cfg.end(); ++i)
    movable_obj_tidy_cfg_[i->id] = *i; 
    
  utils::write_obj_cfg( tidy_cfg,std::string(data_path+"/tidy.cfg") );
  
  return true;
}

////! cost2go()
///*!
//  The cost2go is defined if there is a path from the start vertex to the tmm_goal_
//*/
//double
//PlannerManager::get_cost2go(const TMMVertex& start,const TaskMotionMultigraph& tmm)
//{
//  std::vector<TMMVertex> parents(num_vertices)

//  double cost2go = 0.;
//  
//  TMMVertex v;
//  v = start;
//  do
//  {
//    typename graph_traits<TaskMotionMultigraph>::out_edge_iterator oei, oei_end;
//    tie(oei,oei_end) = out_edges(v,tmm);
//    
//    double cost;
//    cost = get(edge_weight,tmm,*oei);
////    ROS_DEBUG_STREAM("get(edge_weight,tmm,*oei)= " << get(edge_weight,tmm,*oei));
//    
//    cost2go += cost;
//    
//    v = target(*oei,tmm);
////    ROS_DEBUG_STREAM("get(vertex_name,tmm_,v)=" << get(vertex_name,tmm_,v));
//  }
//  while(v != goal);
//    
//  return cost2go;
//}

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
  
  ros::ServiceServer clear_n_ctamp_attempt_srv;
  clear_n_ctamp_attempt_srv = nh.advertiseService("/clear_n_ctamp_attempt", &PlannerManager::clear_n_ctamp_attempt_srv_handle, &pm);
  
  ROS_INFO("PlannerManager: Up and Running ...");
  ros::spin();
  
  return 0;
}
