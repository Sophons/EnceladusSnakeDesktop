#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"


//Create new message object to hold the altered pose
geometry_msgs::PoseStamped poseNew;
ros::Publisher pose_pub;


//Function that sorts the filtered odometry message from 
//robot_localization and spits out another odometry message
//but with a constant position of 0,0,0,0. 
void posefilter(const nav_msgs::Odometry msg)
{
  //Feed the msg's header into the pose's header
  poseNew.header = msg.header;

  //Hardcode the point of the pose to be 0,0,0
  poseNew.pose.position.x = 0;
  poseNew.pose.position.y = 0;
  poseNew.pose.position.z = 0;

  //Feed the msg's orientation into the pose's orientation
  poseNew.pose.orientation.x = msg.pose.pose.orientation.x;
  poseNew.pose.orientation.y = msg.pose.pose.orientation.y;
  poseNew.pose.orientation.z = msg.pose.pose.orientation.z;
  poseNew.pose.orientation.w = msg.pose.pose.orientation.w;

  pose_pub.publish(poseNew);
}


int main(int argc, char **argv)
{
  //Initialise node "posegenerator"
  ros::init(argc, argv, "posegenerator");
  ros::NodeHandle n;

  //Subscribes to the filtered odometry and pushes msg to poseFilter
  ros::Subscriber odom_sub =
	n.subscribe("/odometry/filtered", 1000,  posefilter);

  //Publishes poseNew (the msg) to poseNew (the topic)
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("poseNew",1000);
  ros::Rate loop_rate(100);
  

  //Something to show it's running
  ROS_INFO("Publishing filtered pose to poseNew");

  while( ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}

