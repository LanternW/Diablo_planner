#include <ugv_planner/ugv_manager.h>

void UgvManager::init(ros::NodeHandle &nh)
{
    /*  param  */
    nh.param("ugv/ugv_l",ugv_l,0.6);
    nh.param("ugv/ugv_w",ugv_w,0.4);
    nh.param("ugv/ugv_h",ugv_h,0.3);
    nh.param("ugv/mesh" ,mesh_resource, std::string("package://ugv_planner/param/car.dae"));
    nh.param("ugv/frame",frame,std::string("world"));

    /* callback */
    visugv_pub    = nh.advertise<visualization_msgs::Marker>("odom_mesh", 100,true);
    waypoints_sub = nh.subscribe("waypoints", 1, &UgvManager::rcvWaypointsCallback, this);
    odom_sub      = nh.subscribe("odom", 1, &UgvManager::odomCallback, this);
}

void UgvManager::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    visualization_msgs::Marker WpMarker;
    WpMarker.id               = 0;
    WpMarker.header.stamp     = ros::Time::now();
    WpMarker.header.frame_id  = "world";
    WpMarker.action           = visualization_msgs::Marker::ADD;
    WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
    WpMarker.ns               = "ugv_mesh";
    WpMarker.mesh_use_embedded_materials = true;
    WpMarker.color.r          = 0.0;
    WpMarker.color.g          = 0.0;
    WpMarker.color.b          = 0.0;
    WpMarker.color.a          = 0.0;
    WpMarker.scale.x          = ugv_l/4.5;
    WpMarker.scale.y          = ugv_l/4.5;
    WpMarker.scale.z          = ugv_l/4.5;

    Eigen::Quaterniond q( odom->pose.pose.orientation.w, 
                          odom->pose.pose.orientation.x, 
                          odom->pose.pose.orientation.y,
                          odom->pose.pose.orientation.z );

    Eigen::Quaterniond qz(cos(M_PI/4),0,0,sin(M_PI/4));
    q = qz*q;
    WpMarker.pose.orientation.w = q.w();
    WpMarker.pose.orientation.x = q.x();
    WpMarker.pose.orientation.y = q.y();
    WpMarker.pose.orientation.z = q.z();
    WpMarker.pose.position      = odom->pose.pose.position;
    WpMarker.mesh_resource      = mesh_resource;
    visugv_pub.publish(WpMarker);
}

void UgvManager::rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg)
{
   std::cout<<"manager get waypoints!"<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ugv_vis_node");
  ros::NodeHandle nh("~");
  UgvManager diablo;
  diablo.init(nh);
  ros::spin();
  return 0;
}