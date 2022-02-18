#include <ros/ros.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>

#include <stdlib.h>
#include <time.h>

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
                s_point.z = t_z + (rand() % 10) / 400.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}
void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z  ; t_z < height +ori_z  ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = t_y + offset_y + (rand() % 10) / 100.0 ;
                s_point.z = t_z + (rand() % 10) / 400.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneThickLine(double ori_x, double ori_y, double end_x, double end_y, double ori_z, double end_z, double width)
{
    pcl::PointXYZ  s_point;
    double dist = sqrt( (end_x - ori_x)*(end_x - ori_x) + (end_y - ori_y)*(end_y - ori_y) );
    double x0,y0;
    double verticalx,verticaly;
    double normv;
    verticalx = end_y - ori_y;
    verticaly = ori_x - end_x;
    normv = sqrt(verticalx * verticalx + verticaly * verticaly);
    verticalx /= normv;
    verticaly /= normv;

    for( double t_z = ori_z  ; t_z <= end_z  ; t_z += cloud_resolution/5 )
    {
        for( double t = 0; t <= 1 ; t += cloud_resolution/dist )
        {
            for( double s = -0.5*width; s <= 0.5*width; s += cloud_resolution/2)
            {
                x0 = t * ori_x + (1-t)* end_x;
                y0 = t * ori_y + (1-t)* end_y;
                s_point.x = x0 + s * verticalx + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = y0 + s * verticaly + offset_y + (rand() % 10) / 100.0 ;
                s_point.z = t_z + (rand() % 10) / 400.0 ;

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
                s_point.z = t_z + (rand() % 10) / 400.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneRing(double center_x, double center_y, double height, double radius, double thickness, double ori_ang, double end_ang)
{
    pcl::PointXYZ s_point;
    for(double r = radius; r < radius + thickness; r += cloud_resolution)
    {
        for(double ang = ori_ang ;  ang < end_ang; ang += cloud_resolution * 0.03)
        {
            for(double t_z = 0.0 ;  t_z < height ; t_z += cloud_resolution)
            {
                s_point.x = center_x + r * sin(ang) + offset_x + (rand() % 10) / 100.0 ;
                s_point.y = center_y + r * cos(ang) + offset_y + (rand() % 10) / 100.0  ;
                s_point.z = t_z + (rand() % 10) / 400.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void map1Gene()
{
    geneWall(-0.5,  -19.5    ,   10  , 20, 0.1);
    //wall
    geneWall(-0.5,  0    ,   10  , 0.5, 1.0);
    geneWall(-0.5, -19.5   ,   10  , 0.5, 1.0);
    geneWall(-0.5, -19 ,   0.5 , 19,  1.0);
    geneWall(9   ,  -19 ,   0.5 , 19,  1.0);
    geneWall(0.0 ,  -17   , 9 , 0.5 ,  0.22);
    
    //hole
    double x = 0;
    for(double y = -5; y >= -10; y -= 0.1)
    {
        geneWall(0, y, 3 + sin(x), 0.1, 1.0);
        geneWall(6.0 + sin(x), y, 3.5 - sin(x), 0.1, 1.0);

        geneWall(0.0 ,  y  , 0.5 + 0.1*sin(x) , 9 , 0.1 ,  0.2);
        x += 0.3;
    }

    geneWall(0, -11.5 , 7, 0.5, 1.2);
    geneWall(2, -13 , 7, 0.5, 1.2);
    geneWall(0, -14.5 , 7, 0.5, 1.2);

    //beam
    for(double x = 2; x <= 6 ; x += 2)
    {
        geneWall(x, -13 ,0.5 , 0.5, 2.0, 0.4);

    }
    

    //stamp
    geneWall(2, -5, 1.0, 2.1, 1.1);
    geneWall(3, -3.2, 4.1, 0.4, 1.1);
    geneWall(2, -2, 0.6, 0.6, 1.1);
    geneWall(3.7, -1.4, 0.7, 1.4, 1.1);
    geneWall(5, -1.6, 0.3, 0.5, 0.8);
}

void map2Gene()
{
    //wall
    geneWall(-0.5,  0    ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19.5   ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19 ,   0.5 , 19,  1.0);
    geneWall(10   ,  -19 ,   0.5 , 19,  1.0);

    geneRing(-10, -9.5 ,1.0, 13.14214, 0.5, 0.7854, 3.14159265 * 0.75);
    geneRing(-4, -9.5 ,1.0, 13.14214, 0.5, 0.7854, 3.14159265 * 0.75);

    geneThickLine(0,-6, 7,0, 0.7, 1.0, 0.5);
    //geneThickLine(0,-10, 10,-6, 0.5, 1.0, 0.5);
    geneThickLine(3,-5, 9,-8, 0.0, 0.18, 0.5);
    geneThickLine(2,-17, 9,-15, 0.0, 0.18, 0.5);

    geneRing(9, -10 ,0.18, 3, 0.5, 3.14159265, 3.14159265 * 2);


}

void map3Gene()
{
    //wall
    geneWall(-0.5,  0    ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19.5   ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19 ,   0.5 , 19,  1.0);
    geneWall(10   ,  -19 ,   0.5 , 19,  1.0);


    int gnum = 2;
    srand(time(0));
    double ori_x, ori_y, end_x, end_y;
    while(gnum--)
    {
        ori_x = rand() % 1000 / 100.0;
        ori_y = -rand() % 2000 / 100.0;
        end_x = rand() % 1000 / 100.0;
        end_y = -rand() % 2000 / 100.0;
        geneThickLine(ori_x, ori_y, end_x,end_y, 0.0, 0.18, 0.8);
    }


}

void map4Gene()
{
    //wall
    geneWall(-0.5,  0    ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19.5   ,   11  , 0.5, 1.0);
    geneWall(-0.5, -19 ,   0.5 , 19,  1.0);
    geneWall(10   ,  -19 ,   0.5 , 19,  1.0);


    for(double x = 0 ; x <= 8 ; x +=2)
    {
        for(double y = -2 ; y >= -18 ; y -=2)
        {
            geneWall(x,  y   ,   1.0  , 1.0 , 1.2);
        }
    }
    geneThickLine(0,-6, 9,-13, 0.5, 1.0, 0.5);

}


void pubGlobalMap()
{
    pcl::PointXYZ                     s_point;
    sensor_msgs::PointCloud2          global_map_cloud;


    map2Gene();


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