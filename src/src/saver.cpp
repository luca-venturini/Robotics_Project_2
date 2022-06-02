/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "demo/SavePath.h"
#include "demo/SaveMap.h"

using namespace std;

typedef struct {
  double x;
  double y;
} path_point;

/**
 * @brief Map generation node.
 */
class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free): mapname_(mapname), got_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free), got_path_(false), saved_(false), poses_received(0){
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      path_sub_ = n.subscribe("path", 1, &MapGenerator::pathCallback, this);
      save_path_service_ = n.advertiseService<class MapGenerator, demo::SavePath::Request, demo::SavePath::Response>("save_path", &MapGenerator::savePathServiceCallback, this);
      save_map_service_ = n.advertiseService<class MapGenerator, demo::SaveMap::Request, demo::SaveMap::Response>("save_map", &MapGenerator::saveMapServiceCallback, this);

    }

    bool savePathServiceCallback(demo::SavePath::Request &req, demo::SavePath::Response &res) {
      savePathCallback();
      return true;
    }

    bool saveMapServiceCallback(demo::SaveMap::Request &req, demo::SaveMap::Response &res) {
      saveMapCallback();
      return true;
    }


    void savePathCallback(){
      if (!got_map_ || !got_path_)
          return;
      nav_msgs::Path temp_path = path_;
      nav_msgs::OccupancyGrid temp_map = map_;
      std::vector<path_point> poses_;      

      for(int i = 0; i<temp_path.poses.size(); i++){
        path_point p = {temp_path.poses[i].pose.position.x/temp_map.info.resolution + temp_map.info.width/2, temp_path.poses[i].pose.position.y/temp_map.info.resolution + temp_map.info.height/2};
        poses_.push_back(p);
      }
      int size = temp_path.poses.size()-1;
      double m, m_x, m_y;
      for(int i = 0; i<size; i++){
        m_x = poses_[i+1].x - poses_[i].x > 0 ? poses_[i+1].x - poses_[i].x : poses_[i].x - poses_[i+1].x;
        m_y = poses_[i+1].y - poses_[i].y > 0 ? poses_[i+1].y - poses_[i].y : poses_[i].y - poses_[i+1].y;

        m = m_x > m_y ? m_x : m_y;
        double inc_x = poses_[i+1].x - poses_[i].x;
        double inc_y = poses_[i+1].y - poses_[i].y;
        ROS_INFO("Starting point x: %f - y: %f --- end point x: %f - y: %f", poses_[i].x, poses_[i].y, poses_[i+1].x, poses_[i+1].y);
        for(int j = 1; j<m; j++){
          path_point p = {poses_[i].x + inc_x*j/m, poses_[i].y + inc_y*j/m};
          ROS_INFO("Added point x: %f - y: %f", p.x, p.y);
          poses_.push_back(p);
        }


        ROS_INFO("X: %f - Y: %f", poses_[i].x, poses_[i].y);
      }

      

      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              temp_map.info.resolution, temp_map.info.width, temp_map.info.height);
      for(unsigned int y = 0; y < temp_map.info.height; y++) {
        for(unsigned int x = 0; x < temp_map.info.width; x++) {
          unsigned int i = x + (temp_map.info.height - y - 1) * temp_map.info.width;
          bool find = false;
          for(unsigned int k = 0; k<poses_.size(); k++){
            // ROS_INFO("Actual %f - Map %u", poses_.poses[k].pose.position.x, x);
            if(poses_[k].x < x && poses_[k].x >= x - 1 && poses_[k].y <  (temp_map.info.height - y - 1) && poses_[k].y >=  (temp_map.info.height - y - 1) - 1){
              fputc(184, out);
              find = true;
              break;
            }
          }
          if (!find && temp_map.data[i] >= 0 && temp_map.data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (!find &&  temp_map.data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else if (!find) { //occ [0.25,0.65]
            fputc(105, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      geometry_msgs::Quaternion orientation = temp_map.info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), temp_map.info.resolution, temp_map.info.origin.position.x, temp_map.info.origin.position.y, yaw);

      fclose(yaml);
        
      saved_ = true;
      ROS_INFO("Map savedd");
        
    }

    void saveMapCallback(){
      nav_msgs::OccupancyGrid temp_map = map_;

      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              temp_map.info.resolution, temp_map.info.width, temp_map.info.height);
      for(unsigned int y = 0; y < temp_map.info.height; y++) {
        for(unsigned int x = 0; x < temp_map.info.width; x++) {
          unsigned int i = x + (temp_map.info.height - y - 1) * temp_map.info.width;
          if (temp_map.data[i] >= 0 && temp_map.data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (temp_map.data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      geometry_msgs::Quaternion orientation = temp_map.info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), temp_map.info.resolution, temp_map.info.origin.position.x, temp_map.info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
    }



    void pathCallback(const nav_msgs::Path& path){
      if (!got_map_) return;
      path_ = path;
      got_path_ = true;
    }

    void mapCallback(const nav_msgs::OccupancyGrid& map)
    {
      map_ = map;
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map_.info.width,
               map_.info.height,
               map_.info.resolution);


      ROS_INFO("Done\n");
      got_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    ros::Subscriber path_sub_;
    
    // nav_msgs::Path poses_;
    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path path_;
    bool got_map_;
    int threshold_occupied_;
    int threshold_free_;
    bool got_path_;
    bool saved_;
    int poses_received;
    ros::ServiceServer save_path_service_;
    ros::ServiceServer save_map_service_;

};

#define USAGE ""

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saver");
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  // for(int i=1; i<argc; i++)
  // {
  //   if(!strcmp(argv[i], "-h"))
  //   {
  //     puts(USAGE);
  //     return 0;
  //   }
  //   else if(!strcmp(argv[i], "-f"))
  //   {
  //     if(++i < argc)
  //       mapname = argv[i];
  //     else
  //     {
  //       puts(USAGE);
  //       return 1;
  //     }
  //   }
  //   else if (!strcmp(argv[i], "--occ"))
  //   {
  //     if (++i < argc)
  //     {
  //       threshold_occupied = std::atoi(argv[i]);
  //       if (threshold_occupied < 1 || threshold_occupied > 100)
  //       {
  //         ROS_ERROR("threshold_occupied must be between 1 and 100");
  //         return 1;
  //       }

  //     }
  //     else
  //     {
  //       puts(USAGE);
  //       return 1;
  //     }
  //   }
  //   else if (!strcmp(argv[i], "--free"))
  //   {
  //     if (++i < argc)
  //     {
  //       threshold_free = std::atoi(argv[i]);
  //       if (threshold_free < 0 || threshold_free > 100)
  //       {
  //         ROS_ERROR("threshold_free must be between 0 and 100");
  //         return 1;
  //       }

  //     }
  //     else
  //     {
  //       puts(USAGE);
  //       return 1;
  //     }
  //   }
  //   else
  //   {
  //     puts(USAGE);
  //     return 1;
  //   }
  // }

  // if (threshold_occupied <= threshold_free)
  // {
  //   ROS_ERROR("threshold_free must be smaller than threshold_occupied");
  //   return 1;
  // }

  MapGenerator mg(mapname, threshold_occupied, threshold_free);

  while(ros::ok())
    ros::spinOnce();

  return 0;
}

