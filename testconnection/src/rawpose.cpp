#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"

geometry_msgs::PoseStamped poseRaw;
ros::Publisher rawpose_pub;


void posefilter(const sensor_msgs::Imu msg)
{
  //Feed the msg's header into the pose's header
  poseRaw.header = msg.header;
  poseRaw.header.frame_id = "odom";

  //Hardcode the point of the pose to be 0,0,0
  poseRaw.pose.position.x = 2;
  poseRaw.pose.position.y = 0;
  poseRaw.pose.position.z = 0;

  //Feed the msg's orientation into the pose's orientation
  poseRaw.pose.orientation.x = msg.orientation.x;
  poseRaw.pose.orientation.y = msg.orientation.y;
  poseRaw.pose.orientation.z = msg.orientation.z;
  poseRaw.pose.orientation.w = msg.orientation.w;

  rawpose_pub.publish(poseRaw);
}

int main(int argc, char **argv)
{
  //Initialise node "posegenerator"
  ros::init(argc, argv, "rawposegenerator");
  ros::NodeHandle n;

  //Subscribes to the filtered odometry and pushes msg to poseFilter
  ros::Subscriber data_sub =
        n.subscribe("/BBB2/imu/data", 1000,  posefilter);

  //Publishes poseNew (the msg) to poseNew (the topic)
  rawpose_pub = n.advertise<geometry_msgs::PoseStamped>("poseRaw",1000);
  ros::Rate loop_rate(100);


  //Something to show it's running
  ROS_INFO("Publishing filtered pose to rawpose");

  while( ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}

