#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"

//Initialise Pose msgs for the 3 imu's
geometry_msgs::PoseStamped imu1pose;
geometry_msgs::PoseStamped imu2pose;
geometry_msgs::PoseStamped imu3pose;

//Initialise pose publishers
ros::Publisher imu1pose_pub;
ros::Publisher imu2pose_pub;
ros::Publisher imu3pose_pub;


int main(int argc, char** argv){
	ros::init(argc,argv, "MeasurementModel");
	ros::NodeHandle n;
	ros::Rate rate(100.0);

	tf::TransformListener listener1;
	tf::TransformListener listener2;
	tf::TransformListener listener3;

	imu1pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu1pose",1000);
	imu2pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu2pose",1000);
	imu3pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu3pose",1000);

	geometry_msgs::Point p1, p2, p3;
	geometry_msgs::Quaternion o1, o2, o3;

	//Find transforms for imu1/2/3, and fill their info into imu1/2/3pose
	tf::StampedTransform transform1;
	listener1.waitForTransform("/odom","/imu1", ros::Time(0),ros::Duration(3.0));
	

	tf::StampedTransform transform2;
	listener2.waitForTransform("/odom","/imu2position", ros::Time(0),ros::Duration(3.0));
	
	tf::StampedTransform transform3;
	listener3.waitForTransform("/odom","/imu3", ros::Time(0),ros::Duration(3.0));
	
	while (ros::ok()){
		imu1pose.header.stamp = ros::Time::now();
		imu2pose.header.stamp = ros::Time::now();
		imu3pose.header.stamp = ros::Time::now();
		
		imu1pose.header.frame_id = "odom";
		imu2pose.header.frame_id = "odom";
		imu3pose.header.frame_id = "odom";

		listener1.lookupTransform("/odom","/imu1", ros::Time(0),transform1);
		imu1pose.pose.position.x = transform1.getOrigin().getX();
		imu1pose.pose.position.y = transform1.getOrigin().getY();
		imu1pose.pose.position.z = transform1.getOrigin().getZ();

		imu1pose.pose.orientation.x = transform1.getRotation().getX();
		imu1pose.pose.orientation.y = transform1.getRotation().getY();
		imu1pose.pose.orientation.z = transform1.getRotation().getZ();
		imu1pose.pose.orientation.w = transform1.getRotation().getW();

		imu1pose_pub.publish(imu1pose);
		
		
		listener2.lookupTransform("/odom","/imu2position", ros::Time(0),transform2);
		imu2pose.pose.position.x = transform2.getOrigin().getX();
		imu2pose.pose.position.y = transform2.getOrigin().getY();
		imu2pose.pose.position.z = transform2.getOrigin().getZ();

		imu2pose.pose.orientation.x = transform2.getRotation().getX();
		imu2pose.pose.orientation.y = transform2.getRotation().getY();
		imu2pose.pose.orientation.z = transform2.getRotation().getZ();
		imu2pose.pose.orientation.w = transform2.getRotation().getW();

		imu2pose_pub.publish(imu2pose);



		listener3.lookupTransform("/odom","/imu3", ros::Time(0),transform3);
		imu3pose.pose.position.x = transform3.getOrigin().getX();
		imu3pose.pose.position.y = transform3.getOrigin().getY();
		imu3pose.pose.position.z = transform3.getOrigin().getZ();

		imu3pose.pose.orientation.x = transform3.getRotation().getX();
		imu3pose.pose.orientation.y = transform3.getRotation().getY();
		imu3pose.pose.orientation.z = transform3.getRotation().getZ();
		imu3pose.pose.orientation.w = transform3.getRotation().getW();

		imu3pose_pub.publish(imu3pose);

	}

}
