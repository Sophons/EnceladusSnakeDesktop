#include <cstring>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <random>
//#include <tf/transform_broadcaster.h>

#define PI 3.1415

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_state_pub");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint1_state", 1);
    ros::Rate loop_rate(30);

//    const double degree = M_PI/180;

    // robot state
    double joint1r = 0, joint1p = 0, joint1y = 0;
    double i = 0;

    // message declarations
    sensor_msgs::JointState joint_state;


    while (ros::ok()) {
		//RNG some noise
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(-0.1, 0.1);


        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
		joint_state.position.resize(6);

		joint_state.name[0] = "joint1r";
		joint_state.name[1] = "joint1p";
		joint_state.name[2] = "joint1y";
        joint_state.position[0] = joint1r;
        joint_state.position[1] = joint1p;
        joint_state.position[2] = joint1y;

		joint_state.name[3] = "joint2r";
        joint_state.name[4] = "joint2p";
        joint_state.name[5] = "joint2y";
        joint_state.position[3] = joint1r;
        joint_state.position[4] = joint1p;
        joint_state.position[5] = joint1y;


		//Change the joint values
		joint1p = cos(i*PI/180);
		joint1y = cos(i*PI/180);
		i = i + 1.0;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

