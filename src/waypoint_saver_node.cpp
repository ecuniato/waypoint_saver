#include <waypoint_saver/waypoint_saver_node.h>

static std::string getAnswer()
{
  std::string answer;
  std::cin >> answer;
  // getline(std::cin, answer);
  // if (std::cin.fail() || std::cin.eof())
  // {
  //   std::cin.clear(); // reset cin state
  // }
  return answer;
}

int main(int argc, char **argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "waypoint_saver_node");

  ros::NodeHandle nh, nh_private("~");
  WaypointSaver *waypoint_saver_node = new WaypointSaver(nh, nh_private);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  while (!waypoint_saver_node->isTransformReady())
    ROS_WARN_THROTTLE(3, "Waiting for transforms...");

  ROS_INFO_STREAM("Press 'y' to save current position.");

  while (ros::ok())
  {
    std::string ans = getAnswer();
    if (ans == "y")
      waypoint_saver_node->yamlSaver();
    else if (ans == "c")
    {
      delete waypoint_saver_node;
      exit(0);
    }
    else
      ROS_WARN("No waypoint saved. Press 'y' to save or 'c' to close.");
  }
}

void WaypointSaver::updateTransform()
{
  if (got_odom_ && got_workpiece_)
  {
    transform_mutex_.lock();
    T_Workpiece_Odom_ = T_W_Workpiece_.inverse() * T_W_Odom_;
    transform_mutex_.unlock();
  }
}

// Subscribers
void WaypointSaver::odomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
  // ROS_WARN("got odom");
  Eigen::Vector3d pos(odom_msg->pose.pose.position.x,
                      odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);
  Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  T_W_Odom_.translation() = pos;
  T_W_Odom_.linear() = quat.toRotationMatrix();

  got_odom_ = true;
  updateTransform();
}

void WaypointSaver::workpieceCb(
    const geometry_msgs::TransformStampedConstPtr &workpiece_msg)
{
  // ROS_WARN("got wp");
  Eigen::Vector3d pos(workpiece_msg->transform.translation.x,
                      workpiece_msg->transform.translation.y,
                      workpiece_msg->transform.translation.z);
  Eigen::Quaterniond quat(
      workpiece_msg->transform.rotation.w, workpiece_msg->transform.rotation.x,
      workpiece_msg->transform.rotation.y, workpiece_msg->transform.rotation.z);

  T_W_Workpiece_.translation() = pos;
  T_W_Workpiece_.linear() = quat.toRotationMatrix();

  got_workpiece_ = true;
  updateTransform();
}

bool WaypointSaver::yamlSaver()
{
  std::stringstream pose;

  transform_mutex_.lock();
  Eigen::Vector3d pos = T_Workpiece_Odom_.translation();
  Eigen::Matrix3d rot = T_Workpiece_Odom_.linear();
  transform_mutex_.unlock();
  Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);

  pose << std::endl
       << std::fixed << std::setprecision(3)
       << "\t- {pos: [" << pos(0) << "," << pos(1) << "," << pos(2) << "]"
       << ", att: [" << rpy(0) << "," << rpy(1) << "," << rpy(2)
       << "], force: [0,0,0], stop: True, time: 1.0}";
  traj_file_ << pose.str() << std::flush;

  ROS_INFO_STREAM("New line added: " << pose.str());
  return true;
}
