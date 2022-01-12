#ifndef LOCAL_MAP_MANAGER_H
#define LOCAL_MAP_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <string.h>

#define inf 1>>20
using namespace std;

namespace ugv_planner
{
    struct GridNode;
    typedef GridNode* GridNodePtr;  /*    GridNodePtr：指向struct GridNode结构体类型的指针变量    */

    class GridNode
    {   
    public:
        int id;        // 1--> open set, -1 --> closed set, 0 --> unknown
        Eigen::Vector3d coord; 
        Eigen::Vector2i index;
        
        double cost;
        GridNodePtr father;

        GridNode(Eigen::Vector2i _index, Eigen::Vector3d _coord){  
            id = 0;
            index = _index;
            coord = _coord;

            cost  = 0.0;
            father = NULL;
        }

        GridNode(Eigen::Vector3d _coord, double _cost, Eigen::Vector2i _index, GridNodePtr _father){  
            id = 0;
            index = _index;
            coord = _coord;

            cost  = _cost;
            father = _father;
        }
        GridNode(Eigen::Vector3d _coord, double _cost, Eigen::Vector2i _index, GridNodePtr _father, int _id){  
            id = _id;
            index = _index;
            coord = _coord;

            cost  = _cost;
            father = _father;
        }

        GridNode(){};
        ~GridNode(){};
    };

    class GirdInformation
    {
    public:
        GirdInformation(){is_occupied = false; gap = 10000.0; height = 0.0; roughness = 0.0; plane_normal = Eigen::Vector3d(0,0,0);}
    public:
        bool   is_occupied;             //可通行性
        double gap;                     //落差
        double height;                  //
        double roughness;               //粗糙度
        Eigen::Vector3d plane_normal;   //平面法向量
    };

    class LanGridMapManager
    {
        public:
            LanGridMapManager();
            ~LanGridMapManager();

            void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& global_map);
            void rcvOdomHandler(const nav_msgs::Odometry odom);

            void init(ros::NodeHandle& node_handler);
            bool is_occupied(Eigen::Vector4d posw, int flate);
            bool is_occupiedI(Eigen::Vector2i index, int flate);
            bool is_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate);

            double getGapByI(Eigen::Vector2i index);
            //bool is_occupiedI(Eigen::Vector3i index);
            //bool is_occupied_segment(Eigen::Vector4d pos1 , Eigen::Vector4d pos2);
            //bool is_occupied_segment3(Eigen::Vector3d pos1 , Eigen::Vector3d pos2);

            double getResolution(){return p_grid_resolution;}

            bool indexInMap(int index_x, int index_y);
            bool indexInMap(Eigen::Vector2i index)
            {
                return indexInMap(index(0), index(1));
            }
            bool posInMap(Eigen::Vector4d pos);
            bool posWInMap(Eigen::Vector4d pos);

            double getMountainTop(Eigen::Vector4d posw);
            double getMountainTopIndex(int index_x, int index_y);
            
            Eigen::Vector4d point324(Eigen::Vector3d point3)
            {
                Eigen::Vector4d point4;
                point4.block(0,0,3,1) = point3;
                point4(3) = 1;
                return point4;
            }
            Eigen::Vector3d point423(Eigen::Vector4d point4)
            {
                Eigen::Vector3d point3;
                point3 = point4.block(0,0,3,1);
                return point3;
            }

            Eigen::Vector2i posM2Index(Eigen::Vector4d pos);
            Eigen::Vector2i posW2Index(Eigen::Vector4d pos);
            Eigen::Vector4d index2PosW(Eigen::Vector2i index);
            Eigen::Vector4d posW2posM(Eigen::Vector4d posw)
            {
                posw(1) = -posw(1);
                Eigen::Vector4d posm  = Tm_w * posw;
                return posm;
            }


            Eigen::Vector4d posM2posW(Eigen::Vector4d posm)
            {
                Eigen::Vector4d posw  = Tm_w.inverse() * posm;
                posw(1) = - posw(1);
                return posw;
            }
            
            vector<Eigen::Vector3d> AstarPathSearch(Eigen::Vector3d start, Eigen::Vector3d end);
    

            ////// convex cluster generate
            bool checkConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos);
            vector<Eigen::Vector3d> getNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C);
            vector<Eigen::Vector3d> convexClusterInflation(Eigen::Vector2i seed_index);

        private:

            GirdInformation* grid_information = NULL;
            ros::NodeHandle nh;
            ros::Subscriber global_map_sub;
            ros::Subscriber odometry_sub;
            ros::Publisher  grid_map_pub;


            Eigen::Matrix4d Tm_w;
            nav_msgs::Odometry      ugv_odom;
            nav_msgs::OccupancyGrid grid_map;
            double map_length ;
            double map_width ;
            double p_grid_resolution;


    };
}

#endif