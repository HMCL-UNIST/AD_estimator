
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
  sample_count(0),
  tf_available(false),
  maxQSize_(0),
  gpsPoseQ_(40),
  localPoseQ_(40)   
{
  // temporary variables to retrieve parameters
  double l_to_g_sensor_x, l_to_g_sensor_y, l_to_g_sensor_z, l_to_g_sensor_roll, l_to_g_sensor_pich, l_to_g_sensor_yaw;
  std::string local_sensor_frame, global_sensor_frame;

  nh_.param<double>("l_to_g_sensor_x", l_to_g_sensor_x, 0.0);
  nh_.param<double>("l_to_g_sensor_y", l_to_g_sensor_y, 0.0);
  nh_.param<double>("l_to_g_sensor_z", l_to_g_sensor_z, 0.0);
  
  nh_.param<double>("l_to_g_sensor_roll", l_to_g_sensor_roll, 0.0);
  nh_.param<double>("l_to_g_sensor_pich", l_to_g_sensor_pich, 0.0);
  nh_.param<double>("l_to_g_sensor_yaw", l_to_g_sensor_yaw, 0.0);

  nh_.param<double>("latOrigin", latOrigin, 0.0);
  nh_.param<double>("lonOrigin", lonOrigin, 0.0);
  nh_.param<double>("altOrigin", altOrigin, 0.0);

  nh_.param<bool>("record_transform", record_transform, false);

  
  nh_.param<std::string>("file_name", file_name, "carla_test_v1.csv");

  nh_.param<std::string>("local_sensor_frame", local_sensor_frame, "lidar");
  nh_.param<std::string>("global_sensor_frame", global_sensor_frame, "gnss");
  // Set frame between sensors
  
  try
  {
    ros::Time now = ros::Time(0);
    local_transform_listener.waitForTransform(local_sensor_frame, global_sensor_frame, now, ros::Duration(2.0));
    local_transform_listener.lookupTransform(local_sensor_frame, global_sensor_frame,  ros::Time(0), l_sensor_to_g_sensor);
  }
  catch (tf::TransformException& ex)
  {    
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("sensor frame is not available.. exit node");
    return;
  }

  // set transform between local and global sensors
  Eigen::Translation3f tl_local_to_global(l_to_g_sensor_x, l_to_g_sensor_y, l_to_g_sensor_z);  // tl: translation
  Eigen::AngleAxisf rot_x_local_to_global(l_to_g_sensor_roll, Eigen::Vector3f::UnitX());                   // rot: rotation
  Eigen::AngleAxisf rot_y_local_to_global(l_to_g_sensor_pich, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_local_to_global(l_to_g_sensor_yaw, Eigen::Vector3f::UnitZ());  
  l_to_g_sensor = (tl_local_to_global * rot_z_local_to_global * rot_y_local_to_global * rot_x_local_to_global).matrix();
  g_to_l_sensor = l_to_g_sensor.inverse();
  
  // if not recording, load recorded tf data 
  if(!record_transform && boost::filesystem::exists(file_name)){    
      // set tf by info in file 
      ROS_INFO("TF file loaded");       
      std::vector<std::string> row;
      std::string line, word;
      std::fstream file(file_name, std::ios::in);
      if(file.is_open())
      {
            while(getline(file, line))
            {
            row.clear();
            std::stringstream str(line);
            while(getline(str, word, ',')){
                row.push_back(word);
            }
            tf_data.push_back(row);
            }          
          if(tf_data[0].size() > 3){
            ROS_INFO("Origin info is missing in the file, use value from ros-param");
          }           
      }else{
          ROS_WARN("Could not open file");    
          return;
      }
      latOrigin = std::stod(tf_data[0][0]);  lonOrigin = std::stod(tf_data[0][1]); altOrigin = std::stod(tf_data[0][2]);
      ROS_INFO("GNSS Origin set as, lat: %f, lon: %f, alt: %f",latOrigin,lonOrigin,altOrigin);      
      enu_.Reset(latOrigin,lonOrigin,altOrigin);
  }
  // otherwise, start recording tf 
  else{
    ROS_WARN("Let us compute new tf");
    boost::thread optimizer(&Localtoworld::compute_transform,this); 
  }
  

  worldPosePub = nh_.advertise<geometry_msgs::PoseStamped>("gps_pose_local", 1);
  localGpsPub = nh_.advertise<geometry_msgs::PoseStamped>("local_pose_world", 1);  

  worldGpsSub = nh_.subscribe("gps", 100, &Localtoworld::GpsCallback, this);
  localPoseSub = nh_.subscribe("pose", 100, &Localtoworld::LocalCallback, this);  

  
}

Localtoworld::~Localtoworld()
{}

void Localtoworld::GpsCallback(sensor_msgs::NavSatFixConstPtr fix)
{  
  if (!gpsPoseQ_.pushNonBlocking(fix)){
    ROS_WARN("Poping a GPS measurement due to full queue!!");    
    gpsPoseQ_.popBlocking();    
  }
    
  
  
}

void Localtoworld::LocalCallback(geometry_msgs::PoseStampedConstPtr local_pose)
{  
  if (!localPoseQ_.pushNonBlocking(local_pose)){
    ROS_WARN("Poping a localPose measurement due to full queue!!");
    localPoseQ_.popBlocking();
    }
}

void Localtoworld::compute_transform()
{
  ros::Rate loop_rate(10); // rate of GPS   
  while (ros::ok())
  {
    ROS_WARN("compute tranform");
   sample_count +=1;
   if(sample_count > 3)
   break;
  loop_rate.sleep();
  }
  
  /////////////////////////////////////////////
  //////////       SAVE File       ////////////
  /////////////////////////////////////////////
  std::string file_name_ = file_name;
  while(boost::filesystem::exists(file_name_))
  { 
    ROS_WARN("file already exists");     
    char file_number= file_name_.substr(0, file_name_.find(".csv")).back();
    int version_number = int(file_number)-48; // current version
    if (version_number < 1){ROS_WARN("Pls check the file name carefully, NO file saved");return;} 
    std::string tmp_delimiter = std::to_string(version_number) + ".csv";        
    file_name_ = file_name_.substr(0, file_name_.find(tmp_delimiter))+std::to_string(version_number+1)+".csv";;    
  }   

  ofs.open(file_name_.c_str(), std::ios::app);
  if (!ofs)
  {
    ROS_WARN("Could not open %s",file_name_.c_str());        
  }
  else
  {
    // lat, lon, tf_x, tf_y, tf_z, tf_q_w, tf_q_x, tf_q_y, tf_q_z
    ofs << 0 << "," << 1 << "," << 2<< "," << 3<< "," << 4<< "," << 5<< "," << 6 << "," << 7 << "," << 8 << "," << 9 << std::endl;
    ofs << 1 << "," << 11 << "," << 2<< "," << 3<< "," << 4<< "," << 5<< "," << 6 << "," << 7 << "," << 8 << "," << 9 << std::endl;
    ofs << 2 << "," << 12 << "," << 2<< "," << 3<< "," << 4<< "," << 5<< "," << 6 << "," << 7 << "," << 8 << "," << 9 << std::endl;    
    ofs.close();
    ROS_INFO("File is now saved ");
  }
  

}



int main (int argc, char** argv)
{
ros::init(argc, argv, "LocalToworld");
Localtoworld ltp;
ros::spin();
}
