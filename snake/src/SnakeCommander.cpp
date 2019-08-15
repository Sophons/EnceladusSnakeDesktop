#include <cstring>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <random>
#include <iostream>
#include <cstring>

//#include <tf/transform_broadcaster.h>

#define PI 3.1415
using namespace std;


int main(int argc, char** argv) {
    ros::init(argc, argv, "SnakeCommander");
    ros::NodeHandle n;
    string ns = ros::this_node::getNamespace();
    
    string jointCommandTopic = ns + "/JointCommand";
    string robotStateTopic = ns + "/RobotState";
    ros::Publisher jointCommand_pub = n.advertise<sensor_msgs::JointState>(jointCommandTopic, 100);
    ros::Publisher robotState_pub = n.advertise<std_msgs::String>(robotStateTopic, 100);

    ros::Rate loop_rate(30);


    // robot state
    double joint1r = 0, joint1p = 0, joint1y = 0;

    // message declarations
    sensor_msgs::JointState jointCommand;
	std_msgs::String robotState;
	
	
	jointCommand.position.resize(3);
	jointCommand.velocity.resize(3);
	jointCommand.effort.resize(3);

    while (ros::ok()) {
		string RobotState;
		double archPos, pPos, yPos, archEff, pEff, yEff;
		
		cout << ns;
		cout << "\n Input Robot State: (JointGoalPosition(JGP)/JointPosition(JP)/TorqueCommand(TC)) ";
		cin >> RobotState;
		if (RobotState == "JGP" || RobotState == "JP")
		{
			cout << "\n Input archPos, pPos, yPos: \n";
			cin >> archPos >> pPos >> yPos;
		} else if (RobotState == "TC")
		{
			cout << "\n Input archEff, pEff, yEff: \n";
			cin >> archEff >> pEff >> yEff;
		}
		
		cout << "\n Your inputs were: " << 
				"\n RobotState: " << RobotState << 
				"\n Positions: " << archPos << " , " << pPos << " , "<< yPos <<
				"\n Efforts: " << archEff << " , " << pEff << " , " << yEff;


		//fill robotState
		robotState.data = RobotState;

        //fill jointCommand
        jointCommand.header.stamp = ros::Time::now();

        jointCommand.position[0] = archPos*PI/180;
        jointCommand.position[1] = pPos*PI/180;
        jointCommand.position[2] = yPos*PI/180;

		jointCommand.effort[0] = archEff;
        jointCommand.effort[1] = pEff;
        jointCommand.effort[2] = yEff;

        //send the joint state and transform
        jointCommand_pub.publish(jointCommand);
		robotState_pub.publish(robotState);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

