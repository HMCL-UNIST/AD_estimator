
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


#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "local_to_world.h"

// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.toSec() )


Localtoworld::Localtoworld() :  
  nh_("~"),
  maxQSize_(0),
  gpsPoseQ_(40),
  localPoseQ_(40)   
{
  // temporary variables to retrieve parameters
  double l_to_g_sensor_x, l_to_g_sensor_y, l_to_g_sensor_z, l_to_g_sensor_roll, l_to_g_sensor_pich, l_to_g_sensor_yaw;

  nh_.param<double>("l_to_g_sensor_x", l_to_g_sensor_x, 0.0);
  nh_.param<double>("l_to_g_sensor_y", l_to_g_sensor_y, 0.0);
  nh_.param<double>("l_to_g_sensor_z", l_to_g_sensor_z, 0.0);
  
  nh_.param<double>("l_to_g_sensor_roll", l_to_g_sensor_roll, 0.0);
  nh_.param<double>("l_to_g_sensor_pich", l_to_g_sensor_pich, 0.0);
  nh_.param<double>("l_to_g_sensor_yaw", l_to_g_sensor_yaw, 0.0);

  nh_.param<bool>("record_transform", record_transform, false);

  std::string file_name;
  nh_.param<std::string>("file_name", file_name, "carla_test_v1");


  
  // if not recording, load recorded tf data 
  if(!record_transform){
    ROS_INFO("Loading tf file");
    load_tf_file(file_name);        
    // if (fixedOrigin_)
    // enu_.Reset(latOrigin, lonOrigin, altOrigin);  
  }
  // otherwise, start recording tf 
  else{
    boost::thread optimizer(&Localtoworld::compute_transform,this); 
  }
 
  worldPosePub = nh_.advertise<geometry_msgs::PoseStamped>("gps_pose_local", 1);
  localGpsPub = nh_.advertise<geometry_msgs::PoseStamped>("local_pose_world", 1);  

  worldGpsSub = nh_.subscribe("gps", 100, &Localtoworld::GpsCallback, this);
  localPoseSub = nh_.subscribe("pose", 100, &Localtoworld::LocalCallback, this);  

  
}

Localtoworld::~Localtoworld()
{}

void Localtoworld::load_tf_file(std::string file){
  return;
}

void Localtoworld::GpsCallback(sensor_msgs::NavSatFixConstPtr fix)
{  
  if (!gpsPoseQ_.pushNonBlocking(fix))
    ROS_WARN("Dropping a GPS measurement due to full queue!!");
  
  
}

void Localtoworld::LocalCallback(geometry_msgs::PoseStampedConstPtr local_pose)
{  
  if (!localPoseQ_.pushNonBlocking(local_pose))
    ROS_WARN("Dropping a localPose measurement due to full queue!!");
}

void Localtoworld::compute_transform()
{
  ros::Rate loop_rate(10); // rate of GPS 
  
  while (ros::ok())
  {
    ROS_WARN("compute tranform");
   
  loop_rate.sleep();
  }

}


int main (int argc, char** argv)
{
ros::init(argc, argv, "LocalToworld");
Localtoworld ltp;
ros::spin();
}
