#include <ugv_planner/ugv_manager.h>

void UgvManager::init(ros::NodeHandle &nh)
{
    /*  param  */
    nh.param("ugv/ugv_l",ugv_l,0.6);
    nh.param("ugv/ugv_w",ugv_w,0.4);
    nh.param("ugv/ugv_h",ugv_h,0.3);
    nh.param("ugv/mesh" ,mesh_resource, std::string("package://ugv_planner/param/car.dae"));
    nh.param("ugv/mesh2" ,mesh_resource2, std::string("package://ugv_planner/param/car.dae"));
    nh.param("ugv/frame",frame,std::string("world"));

    /* callback */
    visugv_pub    = nh.advertise<visualization_msgs::Marker>("odom_mesh", 100,true);
    waypoints_sub = nh.subscribe("waypoints", 1, &UgvManager::rcvWaypointsCallback, this);
    odom_sub      = nh.subscribe("odom", 1, &UgvManager::odomCallback, this);
}

//void UgvManager::odomCallback(const nav_msgs::OdometryConstPtr& odom)
//{
//    visualization_msgs::Marker WpMarker;
//    WpMarker.id               = 0;
//    WpMarker.header.stamp     = ros::Time::now();
//    WpMarker.header.frame_id  = "world";
//    WpMarker.action           = visualization_msgs::Marker::ADD;
//    WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
//    WpMarker.ns               = "ugv_mesh";
//    WpMarker.mesh_use_embedded_materials = true;
//    WpMarker.color.r          = 0.0;
//    WpMarker.color.g          = 0.0;
//    WpMarker.color.b          = 0.0;
//    WpMarker.color.a          = 0.0;
//    WpMarker.scale.x          = ugv_l/4.5;
//    WpMarker.scale.y          = ugv_l/4.5;
//    WpMarker.scale.z          = ugv_l/4.5;
//
//    Eigen::Quaterniond q( odom->pose.pose.orientation.w, 
//                          odom->pose.pose.orientation.x, 
//                          odom->pose.pose.orientation.y,
//                          odom->pose.pose.orientation.z );
//
//    Eigen::Quaterniond qz(cos(M_PI/4),0,0,sin(M_PI/4));
//    q = qz*q;
//    WpMarker.pose.orientation.w = q.w();
//    WpMarker.pose.orientation.x = q.x();
//    WpMarker.pose.orientation.y = q.y();
//    WpMarker.pose.orientation.z = q.z();
//    WpMarker.pose.position.x      = odom->pose.pose.position.x;
//    WpMarker.pose.position.y      = odom->pose.pose.position.y;
//    WpMarker.pose.position.z      = odom->pose.pose.position.z;
//    WpMarker.mesh_resource      = mesh_resource;
//    visugv_pub.publish(WpMarker);
//}

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

    Eigen::Matrix3d       R(q);
		double odom_yaw 	    = atan2(R.col(0)[1],R.col(0)[0]);  

    Eigen::Quaterniond qz(cos(-M_PI/4),0,0,sin(-M_PI/4));

    double d = max_height - odom->pose.pose.position.z - 0.2;
    Eigen::Quaterniond qx0(cos(-d/1.5),sin(-d/1.5),0,0);
    Eigen::Quaterniond qx1(cos(d/1.5),sin(d/1.5),0,0);

    double jump_height = odom->twist.twist.angular.x;

    Eigen::Quaterniond q0,q1;
    q0 = qz*q*qx0;
    WpMarker.pose.orientation.w = q0.w();
    WpMarker.pose.orientation.x = q0.x();
    WpMarker.pose.orientation.y = q0.y();
    WpMarker.pose.orientation.z = q0.z();
    WpMarker.pose.position.x      = odom->pose.pose.position.x;
    WpMarker.pose.position.y      = odom->pose.pose.position.y + 0.225 * cos(odom_yaw);
    WpMarker.pose.position.z      = max_height - 0.6 + jump_height;
    WpMarker.mesh_resource      = mesh_resource;
    visugv_pub.publish(WpMarker);


    q1 = qz*q*qx1;
    WpMarker.pose.orientation.w = q1.w();
    WpMarker.pose.orientation.x = q1.x();
    WpMarker.pose.orientation.y = q1.y();
    WpMarker.pose.orientation.z = q1.z();
    WpMarker.mesh_resource      = mesh_resource2;
    WpMarker.id               = 1;
    WpMarker.pose.position.z      = max_height - 0.8 + jump_height;
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