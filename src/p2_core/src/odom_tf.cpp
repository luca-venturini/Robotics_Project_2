#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TfBroad
{
public:
  TfBroad() {
    sub = n.subscribe("/odom", 1000, &TfBroad::callback, this);
  }

  void callback(const nav_msgs::Odometry& msg){
    // set header
    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    // set x,y
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    // set theta
    tf2::Quaternion q;
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x; // q.x();
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y; //q.y();
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z; //q.z();
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w; //q.w();
    // send transform
    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broadcaster");
  TfBroad odom_baselink_tf_broadcaster;
  ros::spin();
  return 0;
}
