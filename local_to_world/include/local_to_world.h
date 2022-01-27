
//   Copyright (c) 2022 Ulsan National Institute of Science and Technology (UNIST)
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

//   Authour : Hojin Lee, hojinlee@unist.ac.kr





#include <sstream>
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <queue>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <GeographicLib/LocalCartesian.hpp>
#include "BlockingQueue.h"
#include <eigen3/Eigen/Geometry>

#define PI 3.14159265358979323846264338


class Localtoworld 
{
  
private:
ros::NodeHandle nh_;
ros::Subscriber worldGpsSub, localPoseSub;
ros::Publisher  worldPosePub, localGpsPub;

BlockingQueue<sensor_msgs::NavSatFixConstPtr> gpsPoseQ_;
BlockingQueue<geometry_msgs::PoseStampedConstPtr> localPoseQ_;

// boost::mutex optimizedStateMutex_;

geometry_msgs::PoseStampedConstPtr gpsPose, localPose;

bool fixedOrigin_;
GeographicLib::LocalCartesian enu_;   /// Object to put lat/lon coordinates into local cartesian
bool gotFirstFix_;
bool record_transform;
double maxGPS_dist_Error_;
int maxQSize_;

static Eigen::Matrix4f l_to_g, g_to_l;
static Eigen::Matrix4f l_to_g_sensor, g_to_l_sensor;

public:
Localtoworld();
~Localtoworld();
void GpsCallback(sensor_msgs::NavSatFixConstPtr fix);
void LocalCallback(geometry_msgs::PoseStampedConstPtr local_pose);

void compute_transform();
void load_tf_file(std::string filename);

};



