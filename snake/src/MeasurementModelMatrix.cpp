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

//Initialize jointAngles: y=rot in y-axis (pitch), z=rot in z-axis (yaw)
double x1 = 0.0, z1 = 0.0; 
double x2 = 0.0, z2 = 0.0;

//Rotation matrices. top left 3*3 is rotation, right column is translation
typedef Matrix<double, 4, 4> H1;
H1 <<cos(z1)	,-cos(x1)*sin(z1)	,cos(z1) + sin(x1)*sin(z1)	,0.0	,
	 sin(z1)	,cos(x1)*cos(z1)	,sin(z1) - sin(x1)*cos(z1)	,0.5	,
	 0.0		,sin(x1)			,cos(x1)					,0.0	,
	 0.0		,0.0				,0.0						,1.0	;

typedef Matrix<double, 4, 4> H2;
H1 <<cos(z2)	,-cos(x2)*sin(z2)	,cos(z2) + sin(x2)*sin(z2)	,0.0	,
	 sin(z2)	,cos(x2)*cos(z2)	,sin(z2) - sin(x2)*cos(z2)	,0.5	,
	 0.0		,sin(x2)			,cos(x2)					,0.0	,
	 0.0		,0.0				,0.0						,1.0	;

//Positions of imu's
typedef Matrix<double 1, 4> imu1;
imu1 << 0.0,
		0.25,
		0.0,
		1.0;

typedef Matrix<double 1, 4> imu2;
imu2 << 0.0,
		0.25,
		0.0,
		1.0;

typedef Matrix<double 1, 4> imu3;
imu3 << 0.0,
		0.25,
		0.0,
		1.0;

//Initialize imu1/2/3position matrices
typedef Matrix<double 1, 4> imu1position;
typedef Matrix<double 1, 4> imu2position;
typedef Matrix<double 1, 4> imu3position;



//Gets called whenever a new joint angle is recieved
void imuMatrix(const sensor_msgs::JointState msg)
{
	//Fill message headers
	imu1pose.header.stamp = ros::Time::now();
	imu2pose.header.stamp = ros::Time::now();
	imu3pose.header.stamp = ros::Time::now();
	
	imu1pose.header.frame_id = "odom";
	imu2pose.header.frame_id = "odom";
	imu3pose.header.frame_id = "odom";
	
		
	//Read and set joint angles. msg.position looks like this:
	//[joint1r, joint1p, joint1y, joint2r, joint2p, joint2y]
	x1 = msg.position[0];
	z1 = msg.position[2];
	x2 = msg.position[3];
	z2 = msg.position[5];

	//Do matrix math for imu1/2/3
	imu1position = imu1;
	imu2position = H1*imu2;
	imu3position = H1*H2*imu3;
	
	//Fill messages
	imu1pose.pose.position.x = imu1position(0,0);
	imu1pose.pose.position.y = imu1position(0,1);
	imu1pose.pose.position.z = imu1position(0,2);
	imu1pose_pub.publish(imu1pose);

	imu2pose.pose.position.x = imu2position(0,0);
	imu2pose.pose.position.y = imu2position(0,1);
	imu2pose.pose.position.z = imu2position(0,2);
	imu2pose_pub.publish(imu2pose);

	imu3pose.pose.position.x = imu3position(0,0);
	imu3pose.pose.position.y = imu3position(0,1);
	imu3pose.pose.position.z = imu3position(0,2);
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
