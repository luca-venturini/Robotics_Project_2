/*
We have taken the map saver implementation from the github page, and we have built 
a node that can save not only the map but also the trajectory, with two different services

THE FILES ARE SAVED IN .ros FOLDER
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
      // Subscribe to read map messages
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      // Subscribe to read path messages
      path_sub_ = n.subscribe("path", 1, &MapGenerator::pathCallback, this);
      // Service to save the path
      save_path_service_ = n.advertiseService<class MapGenerator, p2_core::SavePath::Request, p2_core::SavePath::Response>("save_path", &MapGenerator::savePathServiceCallback, this);
      // Service to save the map
      save_map_service_ = n.advertiseService<class MapGenerator, p2_core::SaveMap::Request, p2_core::SaveMap::Response>("save_map", &MapGenerator::saveMapServiceCallback, this);

    }

    bool savePathServiceCallback(p2_core::SavePath::Request &req, p2_core::SavePath::Response &res) {
      res.old_path_name = mapname_;
      // Set the name for the file to save
      mapname_ = req.new_path_name;
      savePathCallback();
      return true;
    }

    bool saveMapServiceCallback(p2_core::SaveMap::Request &req, p2_core::SaveMap::Response &res) {
      res.old_map_name = mapname_;
      // Set the name for the file to save
      mapname_ = req.new_map_name;
      saveMapCallback();
      return true;
    }


    // Static function that is used to order the points in an array
    static bool comparePoints(path_point lhs, path_point rhs){
        return lhs.y < rhs.y || (!(rhs.y < lhs.y) && lhs.x > rhs.x);       
    }


    // Function to print values, only for debug
    void printPoints(std::vector<path_point> p){
      for (int i = 0; i<p.size(); i++)
        ROS_INFO("index: %i ORDER X: %d - Y: %d", i, p[i].x, p[i].y);
    }


    void savePathCallback(){
      // If no map or path is present, return
      if (!got_map_ || !got_path_)
          return;

      // save in temporary variables the data we will use to build the map/trajectory image
      nav_msgs::Path temp_path = path_;
      nav_msgs::OccupancyGrid temp_map = map_;
      std::vector<path_point> list_poses;      

      // Read the pose from the path, and transform the data in order to have poses with values compatibile with how the map is built
      for(int i = 0; i<temp_path.poses.size(); i++){
        path_point p = {(int) (temp_path.poses[i].pose.position.x/temp_map.info.resolution + temp_map.info.width/2), (int) (temp_path.poses[i].pose.position.y/temp_map.info.resolution + temp_map.info.height/2)};
        list_poses.push_back(p);
      }

      // Interpolation to estiate points between two poses in path message
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

      // Order the array, in this way we reduce the temporal complexity when doing a search, since we will look only at the last element 
      // and then we remove it
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
          // Check if the point is a point of the path
          int i = x + (temp_map.info.height - y - 1) * temp_map.info.width;
          // ROS_INFO("Point x: %d, y: %d", x, y);
          find = false;
          if(list_poses.back().x == x && list_poses.back().y == (temp_map.info.height - y - 1)){
             fputc(104, out);
             find = true;
             while(list_poses.back().x == x && list_poses.back().y == (temp_map.info.height - y - 1))
              list_poses.pop_back();
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saver");
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  MapGenerator mg(mapname, threshold_occupied, threshold_free);

  while(ros::ok())
    ros::spinOnce();

  return 0;
}

