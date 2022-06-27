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
#include "p2_core/SavePath.h"                 
#include "p2_core/SaveMap.h"

using namespace std;

typedef struct {
  int x;
  int y;
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
      save_path_service_ = n.advertiseService<class MapGenerator, p2_core::SavePath::Request, p2_core::SavePath::Response>("save_path", &MapGenerator::savePathServiceCallback, this);
      save_map_service_ = n.advertiseService<class MapGenerator, p2_core::SaveMap::Request, p2_core::SaveMap::Response>("save_map", &MapGenerator::saveMapServiceCallback, this);

    }

    bool savePathServiceCallback(p2_core::SavePath::Request &req, p2_core::SavePath::Response &res) {
      res.old_path_name = mapname_;
      mapname_ = req.new_path_name;
      savePathCallback();
      return true;
    }

    bool saveMapServiceCallback(p2_core::SaveMap::Request &req, p2_core::SaveMap::Response &res) {
      res.old_map_name = mapname_;
      mapname_ = req.new_map_name;
      saveMapCallback();
      return true;
    }

    static bool comparePoints(path_point lhs, path_point rhs){
        return lhs.y < rhs.y || (!(rhs.y < lhs.y) && lhs.x > rhs.x);       
    }

    void printPoints(std::vector<path_point> p){
      for (int i = 0; i<p.size(); i++)
        ROS_INFO("index: %i ORDER X: %d - Y: %d", i, p[i].x, p[i].y);
    }


    void savePathCallback(){
      if (!got_map_ || !got_path_)
          return;
      nav_msgs::Path temp_path = path_;
      nav_msgs::OccupancyGrid temp_map = map_;
      std::vector<path_point> list_poses;      

      for(int i = 0; i<temp_path.poses.size(); i++){
        path_point p = {(int) (temp_path.poses[i].pose.position.x/temp_map.info.resolution + temp_map.info.width/2), (int) (temp_path.poses[i].pose.position.y/temp_map.info.resolution + temp_map.info.height/2)};
        list_poses.push_back(p);
      }
      int size = temp_path.poses.size()-1;
      double m, m_x, m_y;
      for(int i = 0; i<size; i++){
        m_x = list_poses[i+1].x - list_poses[i].x > 0 ? list_poses[i+1].x - list_poses[i].x : list_poses[i].x - list_poses[i+1].x;
        m_y = list_poses[i+1].y - list_poses[i].y > 0 ? list_poses[i+1].y - list_poses[i].y : list_poses[i].y - list_poses[i+1].y;

        m = m_x > m_y ? m_x : m_y;
        double inc_x = list_poses[i+1].x - list_poses[i].x;
        double inc_y = list_poses[i+1].y - list_poses[i].y;
        ROS_INFO("Starting point x: %d - y: %d --- end point x: %d - y: %d", list_poses[i].x, list_poses[i].y, list_poses[i+1].x, list_poses[i+1].y);
        for(int j = 1; j<m; j++){
          path_point p = {(int) (list_poses[i].x + inc_x*j/m), (int) (list_poses[i].y + inc_y*j/m)};
          ROS_INFO("Added point x: %d - y: %d", p.x, p.y);
          list_poses.push_back(p);
        }


        ROS_INFO("X: %d - Y: %d", list_poses[i].x, list_poses[i].y);
      }

      

      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      std::sort(list_poses.begin(), list_poses.end(), comparePoints);
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", temp_map.info.resolution, temp_map.info.width, temp_map.info.height);
      //printPoints(list_poses);
      bool find;
      for(int y = 0; y < temp_map.info.height; y++) {
        for(int x = 0; x < temp_map.info.width; x++) {
          int i = x + (temp_map.info.height - y - 1) * temp_map.info.width;
          // ROS_INFO("Point x: %d, y: %d", x, y);
          find = false;
            // for(unsigned int k = 0; k<list_poses.size(); k++){
            //   // ROS_INFO("Actual %f - Map %u", list_poses.poses[k].pose.position.x, x);
            //   if(list_poses[k].x == x && list_poses[k].y == (temp_map.info.height - y - 1) - 1){
            //     fputc(184, out);
            //     find = true;
            //     std::swap(list_poses[k], list_poses.back());
            //     list_poses.pop_back();
            //     ROS_INFO("k==%d, POINT_X: %d, Y: %d,  --Match found XXXXXXXX", k, list_poses[k].x, list_poses[k].y);
            //     break;
            //   }
            // }
          if(list_poses.back().x == x && list_poses.back().y == (temp_map.info.height - y - 1)){
             fputc(104, out);
             find = true;
             while(list_poses.back().x == x && list_poses.back().y == (temp_map.info.height - y - 1))
              list_poses.pop_back();

             //ROS_INFO("Match found XXXXXXXX %d", (int)list_poses.size());
            //  for (int pp = 0; pp<list_poses.size(); pp++){
            //   ROS_INFO("---index: %d ORDER X: %d - Y: %d", pp, list_poses[pp].x, list_poses[pp].y);
            // }
            //ROS_INFO("Actual point x: %d, y: %d", x, (temp_map.info.height - y - 1));

          }
          if (!find && temp_map.data[i] >= 0 && temp_map.data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (!find &&  temp_map.data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else if (!find) { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancyK data to %s", mapmetadatafile.c_str());
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
  std::string mapname = "mappaa";
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

