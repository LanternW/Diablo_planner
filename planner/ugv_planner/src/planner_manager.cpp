// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" 
#include <cstdlib>
#include <ctime>

//#define DEBUG_TRAJ8

//// for height optimizer
double lanHeightOptimizer::max_height;
double lanHeightOptimizer::min_height;
vector<double> lanHeightOptimizer::ceil_heights;
double lanHeightOptimizer::w_smooth = 100;
double lanHeightOptimizer::w_safty = 4;
////////////////////


const double pi = 3.1415926535;
using namespace quickhull;

namespace ugv_planner
{

  UGVPlannerManager::UGVPlannerManager() {

    has_odom                = false;
    has_target              = false;
    p_max_vel               = 1.0;
    p_max_acc               = 2.0;
    now_pos                 =  now_vel = now_acc = Eigen::Vector3d(0,0,0);
    bezier_basis            = new Bernstein(3); 
    bezier_basis -> setFixedOrder(7);

    lan_bezier_optimizer   = new lanBezierOptimizer();
    lan_height_optimizer   = new lanHeightOptimizer();
    global_map_manager     = new LanGridMapManager();
    global_map_manager     -> init(nh);

    

    corridors.clear();
    ROS_INFO("trajectory planner is ready."); 
  }

  UGVPlannerManager::~UGVPlannerManager() {}

  void UGVPlannerManager::init(ros::NodeHandle& nh)
  {
    this->nh       = nh;
    target_sub     = nh.subscribe("/move_base_simple/goal", 1, &UGVPlannerManager::targetRcvCallback, this);
    odom_sub       = nh.subscribe("/viscar/odom",1 ,&UGVPlannerManager::odomRcvCallback, this);
    des_pos_sub    = nh.subscribe("/ugv_traj_server/despoint",1 ,&UGVPlannerManager::despRcvCallback, this);
    traj_pub       = nh.advertise<traj_utils::Bspline>("trajectory",3);
    target_pub     = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

    vis_render.init(nh);
  }




  void UGVPlannerManager::despRcvCallback(const geometry_msgs::PoseStamped desp)
  {
    des_pos(0) = desp.pose.position.x;
    des_pos(1) = desp.pose.position.y;
    des_pos(2) = desp.pose.position.z;
  }

  void UGVPlannerManager::targetRcvCallback(const geometry_msgs::PoseStamped target_info)
  { 
    if(has_target == false){
        ROS_INFO("Get target");
        has_target = true;
    }
    target     = target_info;
    target_pos(0) = target.pose.position.x;
    target_pos(1) = target.pose.position.y;
    target_pos(2) = 0;
    vis_render.visTarget(target);

    //traj plan
    globalReplan();

  }

  void UGVPlannerManager::odomRcvCallback(const nav_msgs::Odometry odom)
  {
      static Eigen::Vector3d last_vel(0,0,0);
      if(has_odom == false){
        ROS_INFO("Get odometry");
        has_odom = true;
      }
      rt_odometry = odom;
      now_pos(0) = odom.pose.pose.position.x;
      now_pos(1) = odom.pose.pose.position.y;
      now_pos(2) = odom.pose.pose.position.z;
      
      now_vel(0) = odom.twist.twist.linear.x;
      now_vel(1) = odom.twist.twist.linear.y;
      now_vel(2) = odom.twist.twist.linear.z;

      now_acc    = (now_vel - last_vel) * 100;
      last_vel   = now_vel;
  }



  vector<Eigen::Vector3d> UGVPlannerManager::sortVertices(vector<Eigen::Vector3d> vertices)
  {
      Eigen::Vector3d center;
      /*
      for(int i = 0 ; i < vertices.size() ; i++)
      {
        center += vertices[i];
      }

      center = center / vertices.size();
      */
      center = 0.5 * vertices[0] + 0.5 * vertices[1];

      for(int i = 0 ; i < vertices.size() ; i++)
      {
         vertices[i] -= center;
      }

      std::sort(vertices.begin(), vertices.end(), [](Eigen::Vector3d a, Eigen::Vector3d b) {

        double ang1 = atan2(a(1), a(0));
        double ang2 = atan2(b(1), b(0));
        if(ang1 < 0) ang1 += 2 * pi;
        if(ang2 < 0) ang2 += 2 * pi;
        return ang1 < ang2;   
    });

    std::sort(vertices.begin(), vertices.end(), [](Eigen::Vector3d a, Eigen::Vector3d b) {

        double ang1 = atan2(a(1), a(0));
        double ang2 = atan2(b(1), b(0));
        if(ang1 < 0) ang1 += 2 * pi;
        if(ang2 < 0) ang2 += 2 * pi;
        return ang1 < ang2;   
    });
      for(int i = 0 ; i < vertices.size() ; i++)
      {
         vertices[i] += center;
      }
      return vertices;
  }

  void UGVPlannerManager::generateSMC(vector<Eigen::Vector3d> path)
  {
    std::cout<<"[SMC] gene begin, path size = " <<path.size()<<std::endl;
    vector<Eigen::Vector3d> vertices;
    vector<Eigen::Vector3d> vertices_;
    QuickHull<double> quickHull;
    vector<Vector3<double>> grids_set;
    vector<Eigen::Vector3d> grids_set_;
    corridors.clear();

    // path.reserve()
    for(int i = path.size() - 1 ; i >= 0 ; i--)
    {
      if(corridors.size() == 0 || (corridors.size() != 0 && !corridors.back().isPointInPolygon(path[i])) )
      { 
          vertices.clear();
          vertices_.clear();
          grids_set.clear();
          grids_set_.clear();

          Eigen::Vector3d seed_pt = path[i];
          grids_set_ = global_map_manager -> convexClusterInflation(global_map_manager -> posW2Index(global_map_manager ->point324(seed_pt) ));
          //grids_set_ = global_map_manager -> convexClusterInflation(global_map_manager -> posW2Index(Eigen::Vector4d(target_pos(0), target_pos(1),0,1)) );
          std::cout<<"[cluster size = ]" <<grids_set_.size()<<std::endl;
          for(int i = 0 ; i < grids_set_.size(); i++)
          {
            grids_set.push_back(Vector3<double>(grids_set_[i](0), grids_set_[i](1), 0));
          }
          auto hull = quickHull.getConvexHull(grids_set, true, false); 
          auto vertexBuffer = hull.getVertexBuffer();
          int vertex_num = (int)vertexBuffer.size();

          for(int i = 0; i < vertex_num; i++)
          {
              Eigen::Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);  
              vertices.push_back(vertex);
          }
          //vertices = sortVertices(vertices);
          for(int i = 0 ; i < vertices.size(); i++)
          {
            if(  (vertices[i] - seed_pt).norm() > global_map_manager -> getResolution() )
            {
              vertices_.push_back(vertices[i]);
            }
          }
          vertices_ = sortVertices(vertices_);

          //vis_render.renderSMC(vertices_);
          PolygonCorridor new_corridor(vertices_);
          if(i != 0)
          {
            new_corridor.setSeed(path[i-1]);
          }
          corridors.push_back(new_corridor);
          vis_render.renderSMC(corridors);
      }
      else
      {
          //std::cout<<"point in the front corridor" <<std::endl;
          //std::cout<<"PIFC,value = \n" << corridors.back().getDotnum(path[i]) << "\n sum = "<<corridors.back().getDotnum(path[i]).norm()<<std::endl;
          
      }
      //ros::Duration(0.1).sleep();
    }



  }

  void UGVPlannerManager::generateCurveByWps(vector<Eigen::Vector3d> waypoints )
  {
    int seg = waypoints.size() - 1;
    Eigen::VectorXd time     = Eigen::VectorXd::Ones(seg);
    time_allocation.clear();
    for(int i = 0 ; i < seg ; i++)
    {
      time_allocation.push_back(time(i));
    }
    Eigen::MatrixXd bezier_c = Eigen::MatrixXd::Zero(seg, 3*8);
    for(int i = 0 ; i < seg; i++)
    {
        Eigen::Vector3d local_startpt = waypoints[i];
        Eigen::Vector3d local_endpt = waypoints[i+1];
        for(int j = 0 ; j < 8; j++)
        {
            Eigen::Vector3d control_pt = local_startpt + (double(j)/8) * (local_endpt - local_startpt);
            bezier_c(i,j)      = control_pt(0);
            bezier_c(i,j + 8)  = control_pt(1);
            bezier_c(i,j + 16) = control_pt(2);
        }
    }
    //std::cout<<bezier_c<<std::endl;
    vis_render.visBezierTrajectory(bezier_basis, bezier_c, time );
  }


  vector<double> UGVPlannerManager::timeAllocate(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
  {
    double max_vel = 1.5;
    vector<Eigen::Vector3d> waypoints;
    vector<double> time_allocation;
    waypoints.push_back(start_pt);
    for(int i = 1; i < corridors.size(); i++)
    {
      waypoints.push_back(corridors[i].getSeed());
    }
    waypoints.push_back(end_pt);

    for(int i = 0; i < waypoints.size() - 1 ; i++)
    {
      time_allocation.push_back( (waypoints[i+1] - waypoints[i]).norm() / max_vel  );
    }
    return time_allocation;
  }

  void UGVPlannerManager::generateCurveByOptimizer(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int seg)
  {
      Eigen::MatrixXd Qo_u = bezier_basis->getMQM_u();
      Eigen::MatrixXd Qo_l = bezier_basis->getMQM_l();
      Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(2,3);
      Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2,3);
      Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2,3);
      pos.row(0) = start_pt;
      pos.row(1) = end_pt;    
      vel.row(0) << 0.0, 0.0, 0.0;
      acc.row(0) << 0.0, 0.0, 0.0;

      Eigen::VectorXd bezier_time;

      
      time_allocation.clear();
      for(int i = 0 ; i < seg ; i++)
      {time_allocation.push_back(1.0);}
      
      // for ugv, don't use this time_allocation method
      ///time_allocation = timeAllocate(start_pt, end_pt); 

      ros::Time time_before_optimization = ros::Time::now();
        
      int error_code = lan_bezier_optimizer -> bezierCurveGeneration(
                corridors,time_allocation, Qo_u, Qo_l, pos, vel, acc, 7 , 3, 1.0, 0.1);

      ros::Time time_after_optimization = ros::Time::now();
      std::cout<<"[PLANNER MANAGER] optimize done! cost " << time_after_optimization.toSec() - time_before_optimization.toSec() <<" s"<<std::endl;
      std::cout<<"[PLANNER MANAGER] optimizer return value: " << error_code << std::endl;

      bezier_coeff = lan_bezier_optimizer -> getPolyCoeff();
      bezier_time  = lan_bezier_optimizer -> getPolyTime();

      time_allocation = timeAllocate(start_pt, end_pt); 
      time_duration = 0.0;
      for(int i = 0 ; i < time_allocation.size(); i++)
      {
        time_duration += time_allocation[i];
      }
      time_duration   = bezier_time.sum();

      vis_render.visBezierTrajectory(bezier_basis, bezier_coeff, bezier_time );
  }

  
  void UGVPlannerManager::generateHeightCurve()
  {
      double max_vel = 1.5;
      double max_height = 0.7, min_height = 0.2;
      double space_resolution = 0.1;
      vector<Eigen::Vector3d> space_points , space_points_h ,space_points_optimized;
      vector<double> ceil_heights;
      Eigen::Vector3d state, last_state;

      int current_seg   = 0;
      int segment_num   = bezier_coeff.rows();
      //std::cout<<" asdas "<<bezier_coeff<<std::endl;
      double dt = 0.001;
      double t = 0.0;

      last_state = bezier_basis -> getPosFromBezier( bezier_coeff, t, current_seg );
      for( ; current_seg < segment_num; current_seg++)
      {
        t = 0.0;
        while(t <= 1.0)
        {
            state = bezier_basis -> getPosFromBezier( bezier_coeff, t, current_seg );
            if((state - last_state).norm() >= space_resolution)
            {
              space_points.push_back(state); 
              last_state = state;
            }
            t += dt;
        }
      }
      //std::cout<<" spts =  "<<space_points.size()<<std::endl;

      Eigen::Vector2i index;
      Eigen::Vector4d point4;
      Eigen::Vector3d point_h;
      double gap;
      for(int i = 0 ; i < space_points.size(); i++)
      {
        point4 = global_map_manager -> point324(space_points[i]);
        index  = global_map_manager -> posW2Index(point4);
        gap    = global_map_manager -> getGapByI(index);
        gap = (gap > max_height) ? max_height : gap;
        point_h = space_points[i];
        point_h(2) = gap;
        space_points_h.push_back(point_h);

        ceil_heights.push_back(gap);
      }

      //un-smooth curve
      //vis_render.renderPoints(space_points_h , Eigen::Vector3d(0.2,0.9,0.2),1, 0.1, 2);

      int error_code = lan_height_optimizer -> heightCurveGeneration(max_height, min_height,ceil_heights);
      ceil_heights = lan_height_optimizer -> getHeightCps();


      std::cout<<"[PLANNER MANAGER] height optimizer return value: " << error_code << std::endl;
      
      for(int i = 0 ; i < ceil_heights.size(); i++)
      {
        point_h = space_points[i];
        point_h(2) = ceil_heights[i];
        space_points_optimized.push_back(point_h);
      }

      vis_render.renderPoints(space_points_optimized , Eigen::Vector3d(0.2,0.9,0.2),1, 0.04, 3);

      //generate h_bspline
      vector<Eigen::Vector3d> start_end_derivative;
      Eigen::MatrixXd h_bspline_ctrlpts;
      start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
      start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
      start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
      start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));

      UniformBspline::parameterizeToBspline(1, space_points_optimized, start_end_derivative, h_bspline_ctrlpts);
      UniformBspline h_bspline = UniformBspline(h_bspline_ctrlpts, 3, 1);

      vis_render.visHoleBodyTrajectory(h_bspline);
  }

  void UGVPlannerManager::trajPlanning(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
  {
    /*
      vector<Eigen::Vector3d> waypoints;
      waypoints.push_back(start_pt);
      for(int i = 1; i < corridors.size(); i++)
      {
        waypoints.push_back(corridors[i].getSeed());
      }
      waypoints.push_back(end_pt);
      generateCurveByWps(waypoints);
    */
     generateCurveByOptimizer(start_pt, end_pt,corridors.size());
     generateHeightCurve();

  }

  void UGVPlannerManager::globalReplan()
  {
    vector<Eigen::Vector3d> path_0 = global_map_manager -> AstarPathSearch( now_pos, target_pos);
    vis_render.renderPoints(path_0, Eigen::Vector3d(0.6,0.6,0.1),0, 0.05, 1);

    //_polyhedronGenerator -> corridorIncreGeneration(path_0, _poly_array_msg);
    if (path_0.size() > 0)
    {
      generateSMC(path_0);
      trajPlanning( now_pos, target_pos);
    }
    else
    {
      ROS_WARN(" No path! ");
    }
  }












} //namespace