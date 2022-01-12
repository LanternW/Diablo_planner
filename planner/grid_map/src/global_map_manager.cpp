
#include "grid_map/global_map_manager.h"
#include <visualization_msgs/Marker.h>

const double pi = 3.1415926535;



namespace ugv_planner
{

    LanGridMapManager::LanGridMapManager()
    {
        p_grid_resolution    = 0.1;
        map_length           = 20.0;
        map_width            = 10.0;

        Tm_w                 = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd v(pi / 2, Eigen::Vector3d(0, 0, 1));
        Tm_w.block(0,0,3,3) = v.matrix();


    }
    LanGridMapManager::~LanGridMapManager(){}

    void LanGridMapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2& global_map)
    {
        double left_boundary    = -100000.0;
        double right_boundary   = 100000.0;
        double up_boundary      = -100000.0;
        double down_boundary    = 100000.0;

    	pcl::PointCloud<pcl::PointXYZ> global_cloud;
    	pcl::fromROSMsg(global_map, global_cloud);
        Eigen::Vector4d pt_w,  pt_m;

        for(int i = 0 ; i < global_cloud.points.size(); i++)
        {
            if(global_cloud.points[i].x > up_boundary){
                up_boundary = global_cloud.points[i].x;
            }
            if(global_cloud.points[i].x < down_boundary){
                down_boundary = global_cloud.points[i].x;
            }
            if(global_cloud.points[i].y > left_boundary){
                left_boundary = global_cloud.points[i].y;
            }
            if(global_cloud.points[i].y < right_boundary){
                right_boundary = global_cloud.points[i].y;
            }
        }

        map_length = left_boundary - right_boundary;
        map_width  =  up_boundary  - down_boundary;

        Tm_w(0,3)  = -right_boundary;
        Tm_w(1,3)  = -down_boundary;


        grid_map.header.frame_id     = "world";
        grid_map.info.resolution     = p_grid_resolution;
        grid_map.info.height         = int(map_length/p_grid_resolution) + 2 ;
        grid_map.info.width          = int(map_width/p_grid_resolution)+2 ;

        cout<<"[LOCALMAP DEBUG] index = "<<grid_map.info.height << " | " <<grid_map.info.width<<endl;

        pt_w(0) = down_boundary;
        pt_w(1) = right_boundary;
        pt_w(2) = 0;
        pt_w(3) = 1;
        grid_map.info.origin.position.x = pt_w(0);
        grid_map.info.origin.position.y = pt_w(1);
        grid_map.info.origin.position.z = pt_w(2);



        int floor[grid_map.info.width * grid_map.info.height] = {0};   // [0,100]
        //int ceil[grid_map.info.width * grid_map.info.height]  = {0};   // [0,100]

        vector<double> sorted_grid_map[grid_map.info.width * grid_map.info.height];
        if (grid_information != NULL){ delete[] grid_information;}

        grid_information = new GirdInformation[grid_map.info.width * grid_map.info.height];


        int index_x = 0;
        int index_y = 0;
        double new_h = 0;
        for(int i = 0 ; i < global_cloud.points.size(); i++)
        {

            pt_w(0) = global_cloud.points[i].x;
            pt_w(1) = global_cloud.points[i].y;
            pt_w(2) = global_cloud.points[i].z;
            pt_w(3) = 1;
            new_h    = pt_w(2);

            pt_m    = posW2posM(pt_w);
            //pt_m    = posW2posM(Eigen::Vector4d(0,0,0,1));
            
            index_x = pt_m(0) / p_grid_resolution;
            index_y = pt_m(1) / p_grid_resolution;

            sorted_grid_map[index_x * grid_map.info.width + index_y].push_back(new_h);
            //{cout<<"[LOCALMAP DEBUG] index = "<<index_x << " | " <<index_y<<endl;}
            /*
            //h = floor[index_x * grid_map.info.width + index_y] ;
            //c = ceil[index_x * grid_map.info.width + index_y] + 1000;
            if(new_h - h < 20)
            {
                floor[index_x * grid_map.info.width + index_y] = (int(new_h)>100)?100:int(new_h);
                grid_information[index_x * grid_map.info.width + index_y].height = int(new_h);
            }
            else if(new_h < c)
            {
                ceil[index_x * grid_map.info.width + index_y] = int(new_h) - 1000;
                if( ceil[index_x * grid_map.info.width + index_y] + 1000 - floor[index_x * grid_map.info.width + index_y] < 20)
                {
                    floor[index_x * grid_map.info.width + index_y] = ceil[index_x * grid_map.info.width + index_y] + 1000;
                    ceil[index_x * grid_map.info.width + index_y] = 0; 
                }
            }
            */

        }

        // get other grid information here
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            sort(sorted_grid_map[i].begin(), sorted_grid_map[i].end());
            double h = 0;
            for(int j = 0 ; j < sorted_grid_map[i].size(); j++)
            {
                if(sorted_grid_map[i][j] > h && sorted_grid_map[i][j] < h +0.3)
                {
                    h = sorted_grid_map[i][j];
                    floor[i] = (h>1)?100 :h* 100;
                    if(h > 0.2){grid_information[i].is_occupied = true;}
                }
                else if(sorted_grid_map[i][j] >= h + 0.3)
                {
                    grid_information[i].gap = sorted_grid_map[i][j] - h;
                    grid_information[i].height = h;
                    floor[i] = (h>1)?100 :h* 100;
                    if(h > 0.2){grid_information[i].is_occupied = true;}
                    break;
                }
            }
            /*
            grid_information[i].gap = ceil[i] + 1000 - floor[i];
            if(grid_information[i].height > 20){grid_information[i].is_occupied = true;}
            */
        }
        ////

        vector<signed char> a(floor, floor + grid_map.info.width * grid_map.info.height);
        grid_map.data = a;

        grid_map_pub.publish(grid_map);
    }

    void LanGridMapManager::rcvOdomHandler(const nav_msgs::Odometry odom)
    {
        ugv_odom    = odom;
    }


    bool LanGridMapManager::is_occupied(Eigen::Vector4d posw, int flate)
    {
        Eigen::Vector4d pos_m = posW2posM(posw);
        Eigen::Vector4d posm ;
        Eigen::Vector2i index;
        int index_x, index_y;
        for(int i = - flate ; i <= flate; i++)
        {
            for(int j = -flate; j <= flate; j++)
            {
                posm(0) = pos_m(0) + i * p_grid_resolution;
                posm(1) = pos_m(1) + j * p_grid_resolution;
                index = posM2Index(posm);
                index_x = index(0);
                index_y = index(1);
                if(posInMap(posm)) 
                {  
                    return grid_information[index_x * grid_map.info.width + index_y].is_occupied;
                }  
            }
        }

         
        return false;
    }

    bool LanGridMapManager::is_occupiedI(Eigen::Vector2i index, int flate)
    {
        for(int j = -flate; j <= flate; j++)
        {
            for(int i = -flate; i <= flate; i++)
            {
                if(grid_information[(index(0)+i) * grid_map.info.width + (index(1)+j)].is_occupied ){return true;}
            }
        }
        return false;
        //return grid_information[index(0) * grid_map.info.width + index(1)].is_occupied;
        /*
        if(grid_map.data[index(0) * grid_map.info.width + index(1)] > 30) 
        {
            return true;
        }
        return false;
        */
    }

    bool LanGridMapManager::is_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate)
    {
        Eigen::Vector4d posw_inner;
        double distance = (posw1 - posw2).norm();
        int    segs     = 2 * (distance / p_grid_resolution);
        
        for(double t = 1.0/segs ; t < 1 ; t += 1.0/segs)
        {
            posw_inner = t * posw1 + (1-t) * posw2;
            if(is_occupied(posw_inner, flate)){
                return true;
            }
        }
        return false;
    }


    double LanGridMapManager::getGapByI(Eigen::Vector2i index)
    {
        if( indexInMap(index(0),index(1)) )
        {
            return grid_information[index(0)* grid_map.info.width +index(1)].gap;
        }
        else
        {
            ROS_ERROR("[GRID MAP] index out of map!");
            return 1000;
        }
    }

    bool LanGridMapManager::indexInMap(int index_x, int index_y)
    {
        if(index_x >= 0 && index_y >= 0 && index_x < int(map_length/p_grid_resolution) && index_y < int(map_width/p_grid_resolution)) 
        {
            return true; 
        }
        return false;
    }
    bool LanGridMapManager::posInMap(Eigen::Vector4d pos)
    {
        int index_x, index_y;
        index_x = pos(0) / p_grid_resolution;
        index_y = pos(1) / p_grid_resolution;
        return indexInMap(index_x, index_y);
    }

    bool LanGridMapManager::posWInMap(Eigen::Vector4d pos)
    {
        return posInMap(posW2posM(pos));
    }


    Eigen::Vector2i LanGridMapManager::posM2Index(Eigen::Vector4d pos)
    {
        Eigen::Vector2i index;
        index(0) = pos(0) / p_grid_resolution;
        index(1) = pos(1) / p_grid_resolution;
        return index;
    }

    Eigen::Vector2i LanGridMapManager::posW2Index(Eigen::Vector4d pos)
    {
        Eigen::Vector4d posm = posW2posM(pos);
        return posM2Index(posm);
    }

    Eigen::Vector4d LanGridMapManager::index2PosW(Eigen::Vector2i index)
    {
        Eigen::Vector4d posw , posm;
        posm(0) = index(0) * p_grid_resolution + p_grid_resolution/2;
        posm(1) = index(1) * p_grid_resolution + p_grid_resolution/2;
        posm(2) = 0;
        posm(3) = 1;

        posw    = posM2posW(posm);
        return posw;
    }

    void LanGridMapManager::init(ros::NodeHandle& node_handler)
    {
        nh                   = node_handler;
        //p_grid_resolution    = 0.1;
        global_map_sub        = nh.subscribe("global_map",1, &LanGridMapManager::rcvGlobalMapHandler, this);
        odometry_sub         = nh.subscribe("odom", 1 , &LanGridMapManager::rcvOdomHandler, this);
        grid_map_pub         = nh.advertise<nav_msgs::OccupancyGrid>("grid_map",1);

    }

    ///////////////////////////////////////////////////
    ///////////  A* Path search
    ///////////////////////////////////////////////////

    bool isInSet(vector<GridNodePtr> set, int set_id, Eigen::Vector2i pos_index)
    {
        for(int i = 0 ; i < set.size(); i++)
        {
            if(set[i] -> index == pos_index)
            {
                if(set[i] -> id == set_id)
                    return true;
                else
                    return false;
            }
        }
        return false;
    }

    vector<Eigen::Vector3d> LanGridMapManager::AstarPathSearch(Eigen::Vector3d start, Eigen::Vector3d end)
    {
        
        vector<Eigen::Vector3d> path_0;

        if(!posWInMap(point324(start)) || !posWInMap(point324(end)))
        {
            ROS_ERROR("[Astar] boundary points out of map.");
            return path_0;
        }

        int open_set_size = 0 , close_set_size = 0;
        vector<GridNodePtr> set;

        set.push_back( new GridNode(start, 0.0, posW2Index(point324(start)) , NULL , 1) );
        open_set_size ++;

        Eigen::Vector2i end_index = posW2Index(point324(end));

        Eigen::Vector2i ite_index;
        Eigen::Vector3d ite_coord;

        Eigen::Vector2i neighbor_index;
        int neighbor_indx, neighbor_indy;
        double ite_cost, neighbor_cost, heu;

        int iter = 0;

        while(open_set_size > 0)
        {

            iter ++;
            
            ////find minco gridnode
            double minco = 100000;
            int minco_index = 0;
            for(int i = 0 ; i < set.size(); i++)
            {
                if(set[i] -> id != 1) 
                    continue;
                if(set[i] -> cost < minco){
                    minco = set[i] -> cost;
                    minco_index = i;
                }
            }

            GridNodePtr ite_par = set[minco_index];
            ite_par -> id = -1;
            open_set_size  --;
            close_set_size ++;
            //////////////////////////////


            ite_index = ite_par->index;
            ite_coord = ite_par->coord;
            ite_cost  = ite_par->cost;

            if(ite_index == end_index)
            {
                cout<<"[AStar INFO] path found! " << endl;
                break;
            }

            for(int i = -1; i <= 1; i++)
            {
                for(int j = -1; j <= 1; j++)
                {
                    if(i == 0 && j == 0){continue;}
                    neighbor_index(0) = neighbor_indx = ite_index(0) + i;
                    neighbor_index(1) = neighbor_indy = ite_index(1) + j;

                    if( indexInMap(neighbor_indx, neighbor_indy) && 
                       !isInSet(set, 1 , neighbor_index) && 
                       !isInSet(set, -1, neighbor_index) && 
                       !is_occupiedI(neighbor_index,3) )
                    {
                        heu = (end_index - neighbor_index).norm();

                        neighbor_cost = ( (i * j == 0) ? p_grid_resolution : (p_grid_resolution * 1.41)) + ite_cost
                                        + heu * p_grid_resolution ;
                        set.push_back(new GridNode( point423(index2PosW(neighbor_index)),  neighbor_cost,  neighbor_index, (ite_par) ,1) );
                        open_set_size ++;

                    }

                }
            }         
        }

        if(open_set_size == 0)
        {
            ROS_ERROR("[AStar ERR] path not found!");
            set.clear();
            return path_0;
        }
        

        GridNodePtr ite;
        for(int i = set.size() - 1; i >= 0 ; i--)
        {
            if(set[i] -> id == -1)
            {
                ite = set[i];
                break;
            }
        }

        for( ; ite->father != NULL ; ite = ite->father )
        {
            
            path_0.push_back(ite->coord);
        }
        
        set.clear();
        return path_0;
    }


 

    /////////////////////////////////////////////////////
    //////////// convexClusterInflation
    //////////// input : seed pixel , map
    /////////////////////////////////////////////////////
    bool isInSet(vector<Eigen::Vector3d> set, Eigen::Vector3d pos)
    {
        for(int i = 0 ; i < set.size(); i++)
        {
            if(set[i] == pos)
            {
                return true;
            }
        }
        return false;
    }

    vector<Eigen::Vector3d> LanGridMapManager::getNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C)
    {
        vector<Eigen::Vector3d> neighbors;
        Eigen::Vector2i iter_index;
        Eigen::Vector2i neighbor_index;
        for(int i = 0 ; i < set.size(); i++)
        {
            iter_index = posW2Index(point324(set[i]));
            for(int px = -1 ; px <= 1 ; px++)
            {
                for(int py = -1; py <=1 ; py++)
                {
                    if(px == 0 && py == 0){continue;}
                    neighbor_index(0) = iter_index(0) + px;
                    neighbor_index(1) = iter_index(1) + py;
                    if(!indexInMap(neighbor_index) ||
                        isInSet(C_plus, point423(index2PosW(neighbor_index)))    || 
                        isInSet(C, point423(index2PosW(neighbor_index)))         ||
                        isInSet(neighbors, point423(index2PosW(neighbor_index))) ||
                        is_occupiedI(neighbor_index,0)){continue;}
                    neighbors.push_back(point423(index2PosW(neighbor_index)));
                }
            }
        }
        return neighbors;
    }

    bool LanGridMapManager::checkConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos)
    {
        for(int i = 0 ; i < C.size() ; i++)
        {
            //                                 here flate must be 0
            if( is_occupied_line( point324(C[i]) , point324(pos) ,0)) { return false;}
        }
        return true;
    }

    void CPushBack(vector<Eigen::Vector3d>& C, Eigen::Vector3d par ,
                    Eigen::Vector3d &C_north_edge, Eigen::Vector3d &C_south_edge,
                    Eigen::Vector3d &C_east_edge,  Eigen::Vector3d &C_west_edge, vector<Eigen::Vector3d> &C_edge)
    {
        if(par(0) > C_north_edge(0)) C_north_edge = par;
        if(par(0) < C_south_edge(0)) C_south_edge = par;
        if(par(1) > C_west_edge(1))  C_west_edge = par;
        if(par(1) < C_east_edge(1)) C_east_edge = par;

        Eigen::Vector3d v_sw, v_se, v_nw, v_ne;
        v_sw = Eigen::Vector3d(C_south_edge(0), C_west_edge(1) , 0);
        v_se = Eigen::Vector3d(C_south_edge(0), C_east_edge(1) , 0);
        v_nw = Eigen::Vector3d(C_north_edge(0), C_west_edge(1) , 0);
        v_ne = Eigen::Vector3d(C_north_edge(0), C_east_edge(1) , 0);
        C_edge.clear();
        C_edge.push_back(v_sw);
        C_edge.push_back(v_se);
        C_edge.push_back(v_nw);
        C_edge.push_back(v_ne);

        C.push_back(par);
    }

    vector<Eigen::Vector3d> LanGridMapManager::convexClusterInflation(Eigen::Vector2i seed_index)
    {
        vector<Eigen::Vector3d> C , C_plus , C_star, C_star_neighbors;
        Eigen::Vector3d C_north_edge = Eigen::Vector3d(-1e10,0,0);
        Eigen::Vector3d C_south_edge = Eigen::Vector3d(1e10,0,0);
        Eigen::Vector3d C_east_edge = Eigen::Vector3d(0,1e10,0);
        Eigen::Vector3d C_west_edge = Eigen::Vector3d(0,-1e10,0);
        vector<Eigen::Vector3d> C_edge;

        //C.push_back( point423(index2PosW(seed_index)) );
        CPushBack(C, point423(index2PosW(seed_index)) , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
        //get neighbors
        C_plus = getNeighbors(C, C_plus,C);
        while(C_plus.size() != 0)
        {
            for(int i = 0; i < C_plus.size() ; i++)
            {
                if(checkConvexity(C, C_plus[i]))
                //if(checkConvexity(C_edge, C_plus[i]))
                {
                    //C.push_back(C_plus[i]);
                    CPushBack(C, C_plus[i] , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
                    C_star.push_back(C_plus[i]);
                }
            }
            C_plus.clear();
            C_star_neighbors.clear();
            C_star_neighbors = getNeighbors(C_star, C_plus, C);
            for(int i = 0; i < C_star_neighbors.size() ; i++)
            {
                C_plus.push_back(C_star_neighbors[i]);
            }
            C_star.clear();

        }
        return C;
    }

}