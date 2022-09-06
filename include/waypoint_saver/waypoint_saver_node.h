//
// Created by Eugenio Cuniato on 06.09.2022.
//

#ifndef WAYPOINT_SAVER_NODE_H
#define WAYPOINT_SAVER_NODE_H
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <future>
#include <mutex>  // std::mutex
#include <thread> // std::thread

#include <fstream>
#include <ros/package.h>
#include <time.h> /* time_t, struct tm, time, localtime, strftime */

class WaypointSaver
{

public:
  WaypointSaver(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
      : nh_(nh), nh_private_(nh_private)
  {
    got_odom_ = got_workpiece_ = false;

    std::string path = ros::package::getPath("waypoint_saver");
    time_t t = time(0); // get time now
    struct tm *now = localtime(&t);
    char buffer[80];
    strftime(buffer, 80, "%Y-%m-%d.yaml", now);
    traj_file_.open(path + "/resource/" + std::string(buffer),
                    std::ios::out | std::ios::trunc);

    std::ifstream intro_file;
    intro_file.open(path + "/resource/intro_text.yaml", std::ios::in);
    std::stringstream intro_buffer;
    intro_buffer << intro_file.rdbuf();
    intro_file.close();

    traj_file_ << intro_buffer.str() << std::flush;

    odom_sub_ = nh.subscribe("odom_topic", 1, &WaypointSaver::odomCb, this);
    workpiece_sub_ = nh.subscribe("workpiece_transform_topic", 1,
                                  &WaypointSaver::workpieceCb, this);
  }

  ~WaypointSaver() { traj_file_.close(); }

  void updateTransform();

  // Utility
  bool yamlSaver();
  bool isTransformReady()
  {
    return got_odom_ && got_workpiece_;
  }

private:
  // Subscribers
  void odomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void
  workpieceCb(const geometry_msgs::TransformStampedConstPtr &workpiece_msg);
  ros::NodeHandle nh_, nh_private_;

  // Sync
  bool got_odom_, got_workpiece_;

  // Publishers & Subscribers
  ros::Subscriber odom_sub_, workpiece_sub_;

  // Transforms
  Eigen::Affine3d T_W_Odom_, T_W_Workpiece_, T_Workpiece_Odom_;

  // Mutex
  std::mutex transform_mutex_;

  // Files
  std::ofstream traj_file_;
};

#endif // WAYPOINT_SAVER_NODE_H
