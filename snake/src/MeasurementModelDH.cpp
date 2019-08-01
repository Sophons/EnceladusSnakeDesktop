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

//Initialise Pose msgs for the 3 imu's
geometry_msgs::PoseStamped imu1pose;
geometry_msgs::PoseStamped imu2pose;
geometry_msgs::PoseStamped imu3pose;

geometry_msgs::Quaternion imu2orientation;
geometry_msgs::Quaternion imu3orientation;

geometry_msgs::PoseStamped joint1ppose; //Debug use
geometry_msgs::PoseStamped joint1ypose; //Debug use
geometry_msgs::Quaternion joint1pitchOrientation;
geometry_msgs::Quaternion joint1yawOrientation;



//Initialise pose publishers
ros::Publisher imu1pose_pub;
ros::Publisher imu2pose_pub;
ros::Publisher imu3pose_pub;

ros::Publisher joint1ppose_pub; //Debug use
ros::Publisher joint1ypose_pub; //Debug use

//Template for rotation matrices
//Top left 3*3 is rotation, right column is translation
class DHObject{
	public:
	
	double d; 		//offset along previous z-axis between old/new origin
	double theta; 	//angle about previous z-axis between old/new x-axis (our joint angles go here)
	double r; 		//offset along x-axis between old/new origin
	double alpha; 	//angle about previous x-axis between old/new z-axis
	Matrix4d DHMatrix;
	
	//Subfunction for setting theta
	void SetTheta(double x){
		theta = x;
	}
	
	//Subfunction for refreshing the DHMatrix
	void RefreshMatrix(){
		DHMatrix << cos(theta)	, -sin(theta)*cos(alpha), sin(theta)*cos(alpha)	, r*cos(theta)	,
					sin(theta)	, cos(theta)*cos(alpha)	, -cos(theta)*sin(alpha), r*sin(theta)	,
					0.0			, sin(alpha)			, cos(alpha)			, d				,
					0.0			, 0.0					, 0.0					, 1.0			;
	}
	
	//Contructor
	DHObject(double dIn, double thetaIn, double rIn, double alphaIn){
		d = dIn;
		theta = thetaIn;
		r = rIn;
		alpha = alphaIn;
		DHMatrix << cos(theta)	, -sin(theta)*cos(alpha), sin(theta)*cos(alpha)	, r*cos(theta)	,
					sin(theta)	, cos(theta)*cos(alpha)	, -cos(theta)*sin(alpha), r*sin(theta)	,
					0.0			, sin(alpha)			, cos(alpha)			, d				,
					0.0			, 0.0					, 0.0					, 1.0			;
	}
};

//Generate rotation matrices for each joint axis
DHObject link1		(0.0 , 0.0 , 0.5 , 0.0);
DHObject joint1pitch(0.0 , 0.0 , 0.0 , 0.0);
DHObject joint1yaw	(0.0 , 0.0 , 0.0 , -1.57);

DHObject link2		(0.0 , 0.0 , 0.5 , 0.0);
DHObject joint2pitch(0.0 , 0.0 , 0.0 , PI/2);
DHObject joint2yaw	(0.0 , 0.0 , 0.0 , -1*PI/2);

//Positions of imu's
Vector4d imu1(0.25,0.0,0.0,1.0);
Vector4d imu2(0.25,0.0,0.0,1.0);
Vector4d imu3(0.25,0.0,0.0,1.0);

Vector4d joint1pitchOrigin(0.0,0.0,0.0,1.0); //Debug use
Vector4d joint1yawOrigin(0.0,0.0,0.0,1.0); //Debug use

//Initialize imu1/2/3position vectors
Vector4d imu1position(0.0, 0.0, 0.0, 0.0);
Vector4d imu2position(0.0, 0.0, 0.0, 0.0);
Vector4d imu3position(0.0, 0.0, 0.0, 0.0);

Vector4d joint1pitchPosition(0.0, 0.0, 0.0, 0.0); //Debug use
Vector4d joint1yawPosition(0.0, 0.0, 0.0, 0.0); //Debug use

Matrix4d imu2OverallMatrix;
Matrix4d imu3OverallMatrix;


//Altered Quaternion generator from online
geometry_msgs::Quaternion CalculateRotation(Matrix4d a ) {
  geometry_msgs::Quaternion q;
  double trace = a(0,0) + a(1,1) + a(2,2);
  if( trace > 0 ) {
	double s = 0.5 / sqrt(trace + 1.0);
	q.w = 0.25 / s;
	q.x = ( a(2,1) - a(1,2) ) * s;
	q.y = ( a(0,2) - a(2,0) ) * s;
	q.z = ( a(1,0) - a(0,1) ) * s;
  } else {
    if ( a(0,0)  > a(1,1) && a(0,0) > a(2,2) ) {
		  double s = 2.0 * sqrt( 1.0 + a(0,0) - a(1,1) - a(2,2));
		  q.w = (a(2,1) - a(1,2) ) / s;
		  q.x = 0.25 * s;
		  q.y = (a(0,1) + a(1,0) ) / s;
		  q.z = (a(0,2) + a(2,0) ) / s;
    } else if (a(1,1) > a(2,2)) {
		  double s = 2.0 * sqrt( 1.0 + a(1,1) - a(0,0) - a(2,2));
		  q.w = (a(0,2) - a(2,0) ) / s;
		  q.x = (a(0,1) + a(1,0) ) / s;
		  q.y = 0.25 * s;
		  q.z = (a(1,2) + a(2,1) ) / s;
    } else {
		  double s = 2.0 * sqrt( 1.0 + a(2,2) - a(0,0) - a(1,1) );
		  q.w = (a(1,0) - a(0,1) ) / s;
		  q.x = (a(0,2) + a(2,0) ) / s;
		  q.y = (a(1,2) + a(2,1) ) / s;
		  q.z = 0.25 * s;
    }
  }
  return q;
}


//Gets called whenever a new joint angle is recieved
void imuMatrix(const sensor_msgs::JointState msg)
{
//Fill message headers
	imu1pose.header.stamp = msg.header.stamp;
	imu2pose.header.stamp = msg.header.stamp;
	imu3pose.header.stamp = msg.header.stamp;
	joint1ppose.header.stamp = msg.header.stamp; //Debug use
	joint1ypose.header.stamp = msg.header.stamp; //Debug use
	
	
	imu1pose.header.frame_id = "odom";
	imu2pose.header.frame_id = "odom";
	imu3pose.header.frame_id = "odom";
	joint1ppose.header.frame_id = "odom"; //Debug use
	joint1ypose.header.frame_id = "odom"; //Debug use
		
//Read and set joint angles. msg.position looks like this:
//[joint1r, joint1p, joint1y, joint2r, joint2p, joint2y]
	joint1pitch.SetTheta(msg.position[1]);
	joint1yaw.SetTheta(msg.position[2]);
	joint2pitch.SetTheta(msg.position[4]);
	joint2yaw.SetTheta(msg.position[5]);

//Refresh Matrices
	joint1pitch.RefreshMatrix();
	joint1yaw.RefreshMatrix();
	joint2pitch.RefreshMatrix();
	joint2yaw.RefreshMatrix();

////Do matrix math for imu1/2/3
	//imu1
	imu1position = 	imu1;
	
	//imu2
	imu2OverallMatrix = link1.DHMatrix*joint1pitch.DHMatrix*joint1yaw.DHMatrix;
	imu2position = 	imu2OverallMatrix*imu2;
	imu2orientation = CalculateRotation(imu2OverallMatrix);
		
	//imu3
	imu3OverallMatrix = link1.DHMatrix*joint1pitch.DHMatrix*joint1yaw.DHMatrix*
						link2.DHMatrix*joint2pitch.DHMatrix*joint2yaw.DHMatrix;
	imu3position = 	imu3OverallMatrix*imu3;
	imu3orientation = CalculateRotation(imu3OverallMatrix);
	
			//Debugs
			//cout << "\n imu2OverallMatrix = " << imu2OverallMatrix // Debug use
				 //<< "\n imu2orientation = "	  << imu2orientation; // Debug use
			joint1pitchPosition = link1.DHMatrix*joint1pitch.DHMatrix*joint1pitchOrigin; //Debug use
			joint1pitchOrientation = CalculateRotation(link1.DHMatrix*joint1pitch.DHMatrix);
			joint1yawPosition = link1.DHMatrix*joint1pitch.DHMatrix*joint1yaw.DHMatrix*joint1yawOrigin; //Debug use
			joint1yawOrientation = CalculateRotation(link1.DHMatrix*joint1pitch.DHMatrix*joint1yaw.DHMatrix);
	
////Fill messages
	imu1pose.pose.position.x = imu1position(0,0);
	imu1pose.pose.position.y = imu1position(1,0);
	imu1pose.pose.position.z = imu1position(2,0);
	imu1pose_pub.publish(imu1pose);

	imu2pose.pose.position.x = imu2position(0,0);
	imu2pose.pose.position.y = imu2position(1,0);
	imu2pose.pose.position.z = imu2position(2,0);
	imu2pose.pose.orientation = imu2orientation;
	imu2pose_pub.publish(imu2pose);

	imu3pose.pose.position.x = imu3position(0,0);
	imu3pose.pose.position.y = imu3position(1,0);
	imu3pose.pose.position.z = imu3position(2,0);
	imu3pose.pose.orientation = imu3orientation;
	imu3pose_pub.publish(imu3pose);
	
	joint1ppose.pose.position.x = joint1pitchPosition(0,0); //Debug use
	joint1ppose.pose.position.y = joint1pitchPosition(1,0); //Debug use
	joint1ppose.pose.position.z = joint1pitchPosition(2,0); //Debug use
	joint1ppose.pose.orientation = joint1pitchOrientation;
	joint1ppose_pub.publish(joint1ppose); //Debug use

	joint1ypose.pose.position.x = joint1yawPosition(0,0); //Debug use
	joint1ypose.pose.position.y = joint1yawPosition(1,0); //Debug use
	joint1ypose.pose.position.z = joint1yawPosition(2,0); //Debug use
	joint1ypose.pose.orientation = joint1yawOrientation;
	joint1ypose_pub.publish(joint1ypose); //Debug use
	
	//cout << "\n imu1position: " << imu1position  //Debug use
		 //<< "\n imu2position: " << imu2position //Debug use
		 //<< "\n joint1pitch.theta: \n" << joint1pitch.theta //Debug use
		 //<< "\n joint1pitch.DHMatrix: \n" << joint1pitch.DHMatrix //Debug use
		 //<< "\n joint1yaw.theta: \n" << joint1yaw.theta //Debug use
		 //<< "\n joint1yaw.DHMatrix: \n" << joint1yaw.DHMatrix; //Debug use
}


int main(int argc, char** argv){
	ros::init(argc,argv, "MeasurementModel");
	ros::NodeHandle n;
	ros::Rate rate(100.0);

	//Initialise publishers
	imu1pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu1pose",1000);
	imu2pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu2pose",1000);
	imu3pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu3pose",1000);
	
	joint1ppose_pub = n.advertise<geometry_msgs::PoseStamped>("joint1ppose",1000); //Debug use
	joint1ypose_pub = n.advertise<geometry_msgs::PoseStamped>("joint1ypose",1000); //Debug use
	
	//Subscribe to /joint1_state for joint angles
	ros::Subscriber odom_sub = n.subscribe("/joint1_state", 1000,  imuMatrix);

	//cycler
	ros::spin();

}
