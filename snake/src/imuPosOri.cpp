#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv){
	ros::init(argc,argv, "imuPosOri");
	ros::NodeHandle n;
	ros::Rate rate(100.0);

	tf::TransformListener listenerPosition;
	tf::TransformListener listenerOrientation;
	tf::TransformBroadcaster publisherPosOri;


	while (ros::ok()){
		tf::StampedTransform transformPos;
//		ros::Duration(1.0).sleep();
		listenerPosition.waitForTransform("/odom","/imu2position", ros::Time(0),ros::Duration(3.0));
		listenerPosition.lookupTransform("/odom","/imu2position", ros::Time(0),transformPos);


		tf::StampedTransform transformOri;
		listenerOrientation.waitForTransform("/odom","/imu2orientation", ros::Time(0),ros::Duration(3.0));
		listenerOrientation.lookupTransform("/odom","/imu2orientation", ros::Time(0),transformOri);

		tf::Transform transformPosOri;
		tf::Vector3 p = transformPos.getOrigin();
		tf::Quaternion o = transformOri.getRotation();
		transformPosOri.setOrigin(p);
		transformPosOri.setRotation(o);

		publisherPosOri.sendTransform(tf::StampedTransform(transformPosOri, ros::Time::now(), "/odom", "/imu2"));
	}




}
