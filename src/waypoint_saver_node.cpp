#include <waypoint_saver/waypoint_saver_node.h>

int main(int argc, char** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "waypoint_saver_node");

  ros::NodeHandle nh, nh_private("~");
  WaypointSaver waypoint_saver_node(nh, nh_private);
  waypoint_saver_node.run();

  ros::spin();
}

void WaypointSaver::run() {
  
}

// Subscribers
void WaypointSaver::odomCb(const nav_msgs::OdometryConstPtr& odom_msg) {
  Eigen::Vector3d pos(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);

}

void WaypointSaver::workpieceCb(const geometry_msgs::TransformStampedConstPtr& workpiece_msg) {
  Eigen::Vector3d pos(workpiece_msg->transform.translation.x,workpiece_msg->transform.translation.y,workpiece_msg->transform.translation.z);
}

// Utility
bool WaypointSaver::yamlSaver() {
  return true;
}
