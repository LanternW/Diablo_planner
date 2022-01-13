#include <ros/ros.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher global_map_pub;

double offset_x = 0.00;
double offset_y = 0.00;

double cloud_resolution = 0.1;



pcl::PointCloud<pcl::PointXYZ>    global_map_pcl_cloud;

void initParams()
{

}

void geneWall(double ori_x , double ori_y , double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = 0.0; t_z < height ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = t_y + offset_y + (rand() % 10) / 100.0 ;
                s_point.z = t_z + (rand() % 10) / 100.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}
void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z; t_z < height +ori_z  ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = t_y + offset_y + (rand() % 10) / 100.0 ;
                s_point.z = t_z + (rand() % 10) / 100.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneTrangle(double ori_x , double ori_y, double height, double depth, double length)
{
    pcl::PointXYZ s_point;
    for(double t_x = ori_x; t_x < ori_x + depth; t_x += cloud_resolution)
    {
        for(double t_y = ori_y ;  t_y < ori_y + length; t_y += cloud_resolution)
        {
            for(double t_z = 0.0 ;  t_z < (length - t_y + ori_y)*height/length ; t_z += cloud_resolution)
            {
                s_point.x = t_x + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = t_y + offset_y + (rand() % 10) / 100.0  ;
                s_point.z = t_z + (rand() % 10) / 100.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}


void pubGlobalMap()
{
    pcl::PointXYZ                     s_point;
    sensor_msgs::PointCloud2          global_map_cloud;



    geneWall(-0.5,  -19.5    ,   10  , 20, 0.1);
    //wall
    geneWall(-0.5,  0    ,   10  , 0.5, 1.0);
    geneWall(-0.5, -19.5   ,   10  , 0.5, 1.0);
    geneWall(-0.5, -19 ,   0.5 , 19,  1.0);
    geneWall(9   ,  -19 ,   0.5 , 19,  1.0);
    geneWall(0.0 ,  -17   , 9 , 0.5 ,  0.2);
    
    //hole
    geneWall(0, -5, 4, 0.5, 0.5);
    geneWall(5, -5, 4, 0.5, 0.5);
    geneWall(0, -5, 0.6, 9, 0.5, 0.3);


    geneWall(0, -5.5, 4, 0.5, 0.5);
    geneWall(5.5, -5.5, 4, 0.5, 0.5);
    geneWall(0, -5.5, 0.4, 9, 0.5, 0.5);

    geneWall(0, -8, 7, 0.5, 0.5);
    geneWall(8, -8, 1, 0.5, 0.5);
    geneWall(0, -8, 0.5, 9, 0.5, 0.4);

    //stamp
    geneWall(2, -2, 0.6, 0.6, 1.1);
    geneWall(3.7, -1.4, 0.7, 1.4, 1.1);
    geneWall(5, -1.6, 0.3, 0.5, 0.8);
    geneWall(4, -3, 0.4, 0.2, 1.1);


/*
    geneWall(7.0, -11 , 2, 0.5, 0.2);
    geneWall(4.0, -11 , 2, 0.5, 0.2);
    geneWall(0.0, -10 , 9, 0.5, 0.2);
    geneWall(4.0, -10 , 2, 0.5, 0.4);


    geneWall(0, -7 , 7, 0.5, 0.4);
    geneTrangle(6, -6.5, 0.4, 1, 2);


    geneWall(2.0, -3 , 1, 1, 1.7);
    geneWall(6.0, -3 , 1, 1, 1.7);
    geneWall(4.0, -6 , 1, 1, 1.7);

    geneWall(4.0, -9 , 1, 1, 1.7);
    geneWall(4.0, -11 , 1, 1, 1.7);
    geneWall(4.5, -13 , 1, 1, 1.7);
    geneWall(2.0, -8.5 , 1, 1, 1.7);
    geneWall(3.0, -14.5 , 1, 1, 1.7);

    geneWall(7.0, -12.5 , 2, 1, 1.7);
    geneWall(8.0, -7.5 , 2.5, 1.5, 1.7);


    geneWall(3.0, 9 , 1, 1, 1.7);
    */



    pcl::toROSMsg(global_map_pcl_cloud, global_map_cloud);
    global_map_cloud.header.frame_id = "world";
    global_map_pub.publish(global_map_cloud);
    
    ROS_INFO("global map published! ");
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "tmap_generator"); 
    ros::NodeHandle nh; 

    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5); 

    int t = 3;
    while(t--)
    {
        pubGlobalMap();
        ros::Duration(1.0).sleep();
    }
    ros::spin();
    return 0;
}