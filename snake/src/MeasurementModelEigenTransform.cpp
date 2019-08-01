#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Eigen>

#define PI 3.1415

using namespace Eigen;
using namespace std;

////Initialise all needed variables/parameters
	//Initialise Pose msgs for the 3 imu's
	geometry_msgs::PoseStamped imu1pose;
	geometry_msgs::PoseStamped imu2pose;
	geometry_msgs::PoseStamped imu3pose;

	//Initialise pose publishers	
	ros::Publisher imu1pose_pub;
	ros::Publisher imu2pose_pub;
	ros::Publisher imu3pose_pub;
	
	Vector4d imu1(0.25,0.0,0.0,1.0);
	Vector4d imu2(0.25,0.0,0.0,1.0);
	Vector4d imu3(0.25,0.0,0.0,1.0);

	Vector4d imu1Position(0.0,0.0,0.0,0.0);
	Vector4d imu2Position(0.0,0.0,0.0,0.0);
	Vector4d imu3Position(0.0,0.0,0.0,0.0);

//Gets called whenever a new joint angle is recieved
void imuMatrix(const sensor_msgs::JointState msg)
{
////Fill message headers
	imu1pose.header.stamp = msg.header.stamp;
	imu2pose.header.stamp = msg.header.stamp;
	imu3pose.header.stamp = msg.header.stamp;

	imu1pose.header.frame_id = "odom";
	imu2pose.header.frame_id = "odom";
	imu3pose.header.frame_id = "odom";
	
////Do matrix math for imu1/2/3
	//imu1
	imu1Position = 	imu1;
	
	//imu2
	Transform<double,3,Affine> imu2Transform 
		= Translation3d(Vector3d(0.5,0.0,0.0))
		* AngleAxisd(msg.position[1], Vector3d::UnitY())
		* AngleAxisd(msg.position[2], Vector3d::UnitZ());
	imu2Position = imu2Transform*imu2;
	Quaternion<double> imu2Orientation(imu2Transform.rotation()); 
	
	//		cout << "\n imu2PositionB: \n " << imu2PositionB << "\n"; //Debug use
	//		cout << "\n imu2Orientation: \n" << imu2Orientation << "\n"; //Debug use
	
	//imu3
	Transform<double,3,Affine> imu3Transform 
		= Translation3d(Vector3d(0.5,0.0,0.0))
		* AngleAxisd(msg.position[1], Vector3d::UnitY())
		* AngleAxisd(msg.position[2], Vector3d::UnitZ())
		* Translation3d(Vector3d(0.5,0.0,0.0))
		* AngleAxisd(msg.position[1], Vector3d::UnitY())
		* AngleAxisd(msg.position[2], Vector3d::UnitZ());
	imu3Position = imu3Transform*imu3;
	Quaternion<double> imu3Orientation(imu3Transform.rotation()); 
	
//Fill messages
	imu1pose.pose.position.x = imu1Position(0,0);
	imu1pose.pose.position.y = imu1Position(1,0);
	imu1pose.pose.position.z = imu1Position(2,0);
	imu1pose_pub.publish(imu1pose);

	imu2pose.pose.position.x = imu2Position(0,0);
	imu2pose.pose.position.y = imu2Position(1,0);
	imu2pose.pose.position.z = imu2Position(2,0);
	imu2pose.pose.orientation.x = imu2Orientation.x();
	imu2pose.pose.orientation.y = imu2Orientation.y();
	imu2pose.pose.orientation.z = imu2Orientation.z();
	imu2pose.pose.orientation.w = imu2Orientation.w();
	imu2pose_pub.publish(imu2pose);

	imu3pose.pose.position.x = imu3Position(0,0);
	imu3pose.pose.position.y = imu3Position(1,0);
	imu3pose.pose.position.z = imu3Position(2,0);
	imu3pose.pose.orientation.x = imu3Orientation.x();
	imu3pose.pose.orientation.y = imu3Orientation.y();
	imu3pose.pose.orientation.z = imu3Orientation.z();
	imu3pose.pose.orientation.w = imu3Orientation.w();
	imu3pose_pub.publish(imu3pose);
}


int main(int argc, char** argv){
	ros::init(argc,argv, "MeasurementModel");
	ros::NodeHandle n;
	ros::Rate rate(100.0);

	//Initialise publishers
	imu1pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu1pose",1000);
	imu2pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu2pose",1000);
	imu3pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu3pose",1000);

	//Subscribe to /joint1_state for joint angles
	ros::Subscriber odom_sub = n.subscribe("/joint1_state", 1000,  imuMatrix);

	//cycler
	ros::spin();

}
