//
// Created by Eugenio Cuniato on 06.09.2022.
//

#ifndef WAYPOINT_SAVER_NODE_H
#define WAYPOINT_SAVER_NODE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <math.h>

#include <thread>         // std::thread
#include <mutex>          // std::mutex

#include <ros/package.h>
#include <fstream>
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */

class WaypointSaver {

 public:
  WaypointSaver(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
      nh_(nh),
      nh_private_(nh_private) {
        got_odom_ = got_workpiece_ = false;

        std::string path = ros::package::getPath("waypoint_saver");
        time_t t = time(0);   // get time now
        struct tm * now = localtime( & t );
        char buffer [80];
        strftime(buffer,80,"%Y-%m-%d.yaml",now);
        traj_file_.open(path+"/resource/"+std::string(buffer), std::ios::out | std::ios::trunc);

        std::ifstream intro_file;
        intro_file.open(path+"/resource/intro_text.yaml",std::ios::in);
        std::stringstream intro_buffer;
        intro_buffer << intro_file.rdbuf();
        intro_file.close();
        
        traj_file_ << intro_buffer.str();
  
  }

  ~WaypointSaver() {
    traj_file_.close();
  }

  void run();

  // Subscribers
  void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
  void workpieceCb(const geometry_msgs::TransformStampedConstPtr& workpiece_msg);

  // Utility
  bool yamlSaver();

 private:
  ros::NodeHandle nh_, nh_private_;

  // Sync
  bool got_odom_, got_workpiece_;

  // Publishers & Subscribers
  ros::Subscriber sub_pointcloud_;

  // Transforms
  Eigen::Affine3d T_W_Odom_, T_W_Workpiece_, T_Workpiece_Odom_;

  // Mutex
  std::mutex transform_mutex_; 

  // Files
  std::ofstream traj_file_;

};

#endif //TOF_NORMAL_ESTIMATION_NORMAL_ESTIMATION_NODE_H
