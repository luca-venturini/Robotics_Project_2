#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

class ShowPath
{
public:

  /*
  We use this class to get poses and add them to our path, that will be then published, in order to build the trajectory
  */
  ShowPath() {
    pub = n.advertise<nav_msgs::Path>("path", 1000);
    sub = n.subscribe("/amcl_pose", 1000, &ShowPath::callback, this);
  }

  void callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    path.header.stamp = msg.header.stamp;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg.header;
    pose_stamped.pose = msg.pose.pose;
    path.poses.push_back(pose_stamped);
    pub.publish(path);
  }

private:
  nav_msgs::Path path;
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "path_broadcaster");
  ShowPath path_broadcaster;
  ros::spin();
  return 0;
}
