#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "TrajPlanner.h"
#include "CaseyControl.h"
#include <iostream>
#include <cstring>
#define PI 3.1415

using namespace std;

//Initialize some global memories
double pitchCurrent = 0.0;
double yawCurrent = 0.0;
double maxVelocity = 0.5;
double period = 0.01;
double rate = 100.0;
bool newCommandSent = false;
int robotStateNum = 1;



//Initialize some messages
//std_msgs::String RobotState;
sensor_msgs::JointState mostRecentCommand;
sensor_msgs::JointState JointState;

//Gets called whenever a new joint command is recieved
void JointCommand(const sensor_msgs::JointState CommandMsg)
{//Rmb that we got a new command, and store that msg into mostRecentCommand
	newCommandSent = true;
	mostRecentCommand = CommandMsg;
	ROS_INFO_STREAM("\n New command is:" << 
					"\n xPos = " << mostRecentCommand.position[0] <<
					"\n yPos = " << mostRecentCommand.position[1] <<
					"\n zPos = " << mostRecentCommand.position[2] <<
					"\n xEff = " << mostRecentCommand.effort[0]   <<
					"\n yEff = " << mostRecentCommand.effort[1]   <<
					"\n zEff = " << mostRecentCommand.effort[2]		);
}

//Gets called whenever a new robot state is recieved
void RobotStateSetter(const std_msgs::String RobotStateMsg)
{//Store our new robot state
	if(RobotStateMsg.data == "JointGoalPosition")	{ robotStateNum = 1; }
	if(RobotStateMsg.data == "JointPosition")		{ robotStateNum = 2; }
	if(RobotStateMsg.data == "TorqueCommand")		{ robotStateNum = 3; }
	ROS_INFO_STREAM("\n Now in Robot State " << RobotStateMsg.data << robotStateNum );
}

string JointNameChanger(string ns, int i){ //Sets the joint names depending on namespace
	if(ns == "/BBB1"){
		switch(i){
			case 1:{ return "joint1p"; }
			case 2:{ return "joint1y"; }
			}
	} else if(ns == "/BBB2"){
		switch(i){
			case 1:{ return "joint2p"; }
			case 2:{ return "joint2y"; }
			}
	} else if(ns == "/BBB3"){
		switch(i){
			case 1:{ return "joint3p"; }
			case 2:{ return "joint3y"; }
			}
	} else if(ns == "/BBB4"){
		switch(i){
			case 1:{ return "joint4p"; }
			case 2:{ return "joint4y"; }
			}
	} else {
		ROS_WARN("Invalid namespace!");
	}
}

void populateMsgs(){ //Used to avoid segfault, initiallizes the messages
	mostRecentCommand.position.resize(3);
	mostRecentCommand.velocity.resize(3);
	mostRecentCommand.effort.resize(3);

	JointState.name.resize(3);
	JointState.position.resize(3);
	JointState.velocity.resize(3);
	JointState.effort.resize(3);
	
	for(int i = 0; i < 3; ++i){
		mostRecentCommand.position[i] = 0.0;
		mostRecentCommand.velocity[i] = 0.0;
		mostRecentCommand.effort[i] = 0.0;
		
		JointState.position[i] = 0.0;
		JointState.velocity[i] = 0.0;
		JointState.effort[i] = 0.0;
		
	}
}


int main(int argc, char** argv){
	ros::init(argc,argv, "BBBMain");
	ros::NodeHandle n;
	ros::Rate loop_rate(rate);

	//Get current namespace
	string ns = ros::this_node::getNamespace();
	if (ns == "//BBB1"){ ns = "/BBB1"; }
	if (ns == "//BBB2"){ ns = "/BBB2"; }
	if (ns == "//BBB3"){ ns = "/BBB3"; }
	if (ns == "//BBB4"){ ns = "/BBB4"; }
	ROS_INFO_STREAM("\n namespace is: " << ns);

	//Default is that we don't have a new message and that we aren't running a traj
	newCommandSent = false;
	bool runningTrajectory = false;
	bool pitchMotionCompleted = true;
	bool yawMotionCompleted = true;

	//Subscribe to /JointCommand for new movement commands
	//Subscribe to /RobotState for movement method setting
	string jointCommandTopic = ns + "/JointCommand";
    string robotStateTopic = ns + "/RobotState";
	ros::Subscriber JointCommand_sub = n.subscribe(jointCommandTopic, 1000,  JointCommand);
	ros::Subscriber RobotState_sub = n.subscribe(robotStateTopic, 1000,  RobotStateSetter);
	
	//Set up publisher for joint data
	ros::Publisher JointState_Pub = n.advertise<sensor_msgs::JointState>("/JointState",1000);
	
	//Initialize TrajectoryPlanner objects for Pitch and Yaw
	TrajectoryPlanner PitchPlanner;
	TrajectoryPlanner YawPlanner;
	
	//Startup Casey's controller
	CaseyControl CaseyController;
	CaseyController.FillZeroes();

	//Initialize empty messages so maybe segfault goes away :)))))
	populateMsgs();
	
	//Change 

	string pitchName = JointNameChanger(ns,1);
	string yawName = JointNameChanger(ns,2);
	
	//Keep looping our pubs/subs/setters while ros runs!
	while(ros::ok()){
		
		loop_rate.sleep();
		ros::spinOnce();

		//Publish joints state information here!!!
		JointState.header.stamp = ros::Time::now();
		JointState.header.frame_id = ns;

		JointState.name[1] = pitchName;
		JointState.name[2] = yawName;
		
		JointState.position[0] = CaseyController.getPosition(0);
		JointState.position[1] = CaseyController.getPosition(1);
		JointState.position[2] = CaseyController.getPosition(2);
		
		JointState.velocity[0] = CaseyController.getVelocity(0);
		JointState.velocity[1] = CaseyController.getVelocity(1);
		JointState.velocity[2] = CaseyController.getVelocity(2);
		
		JointState.effort[0] = CaseyController.getEffort(0);
		JointState.effort[1] = CaseyController.getEffort(1);
		JointState.effort[2] = CaseyController.getEffort(2);

		JointState_Pub.publish(JointState);
		
		//If no new command, and trajectory is not in progress, go thru
		//otherwise pause here
		if(!newCommandSent && !runningTrajectory){
			ROS_INFO_STREAM("\n Checking newCommand@@runningTraj" << 
							"\n newCommand = " << newCommandSent << " runningTrajectory = " << runningTrajectory
							);
			continue;
		}
	
		//If new command, clear past traj, set runningTraj false, set pitch/yawCurrent
		if(newCommandSent){ 
			ROS_INFO_STREAM("\n new command! doing newCommand segment");
			////Clear past Traj
				PitchPlanner.clearAll();
				YawPlanner.clearAll();
			////Set runningTraj to false
				runningTrajectory = false;
			////Set current position as pitchCurrent/yawCurrent
				pitchCurrent = CaseyController.getPosition(1);
				yawCurrent = CaseyController.getPosition(2);
		}
		
		//Set torque for archimedes screw here!!!
		CaseyController.setEffort(0,mostRecentCommand.effort[0]);
			ROS_INFO_STREAM("\n Starting switch segment");		
		//Depending on ROBOT_STATE, set points/torques into Casey's controller
		//This switch state only matters for the pitch and yaw
		switch(robotStateNum){
			case 1:{

				ROS_INFO_STREAM("\n Starting Case 1");
				double pitchCommand;
				double yawCommand;
				
				//if we get a new command msg, set variables, make new trajectory,
				//and rmb that we are in motion
				if(newCommandSent){
					//Recieve new joint angles from the most recent command
					pitchCommand = mostRecentCommand.position[1];
					yawCommand = mostRecentCommand.position[2];

					//Run TrajectoryPlanner to calculate the trajectories
					PitchPlanner.setTrajectory(pitchCurrent, pitchCommand, maxVelocity, period);
					YawPlanner.setTrajectory(yawCurrent, yawCommand, maxVelocity, period);
					
					//You are about to run a trajectory! Motion is not completed
					pitchMotionCompleted = false;
					yawMotionCompleted = false;
					ROS_INFO_STREAM("\n new command recieved! pitchCommand = " << pitchCommand << " yawCommand = " << yawCommand);
				}
				
				//if pitching or yawing isn't done yet, get/set next point till motion is done
				if (!pitchMotionCompleted || !yawMotionCompleted){ ROS_INFO_STREAM("\n seeing if pitch or yaw is done");
					if(!pitchMotionCompleted){ ROS_INFO_STREAM("\n pitchMotionCompleted = " << pitchMotionCompleted);
						//If pitching in progress, get the next pitch point from traj 
						//and feed it to Casey's controller.
						double nextPitchPoint = PitchPlanner.getNextTrajPoint();
						CaseyController.setPosition(1,nextPitchPoint);
						ROS_INFO_STREAM(" next pitch point is: " << nextPitchPoint);

						//If we hit the last point, we finished the motion
						if(nextPitchPoint == pitchCommand){ ROS_INFO_STREAM("\n ####pitchMotionCompleted####");
							pitchMotionCompleted = true;
						}
					}
					
					if(!yawMotionCompleted){ 
						ROS_INFO_STREAM("\n yawMotionCompleted = " << yawMotionCompleted);
						//If yawing in progress, get the next yaw point from traj 
						//and feed it to Casey's controller.
						double nextYawPoint = YawPlanner.getNextTrajPoint(); 
						CaseyController.setPosition(2,nextYawPoint);
						ROS_INFO_STREAM(" next yaw point is: " << nextYawPoint);
						//If we hit the last point, we finished the motion
						if(nextYawPoint == yawCommand){ROS_INFO_STREAM("\n ####yawMotionCompleted####");
							yawMotionCompleted = true;
						}
					}
				ROS_INFO_STREAM("\n did traj segment");
				}
				
				//rmb that we are still running a trajectory motion!
				runningTrajectory = !pitchMotionCompleted || !yawMotionCompleted;
				ROS_INFO_STREAM("\n runningTrajectory = " << runningTrajectory);
				break;
			}
			case 2:{ ROS_INFO_STREAM("\n Starting Case 2");
				//get and set the position commands into Casey's controller
				double pitchPosition = mostRecentCommand.position[1];
				double yawPosition = mostRecentCommand.position[2];

				CaseyController.setPosition(1,pitchPosition);
				CaseyController.setPosition(2,yawPosition);
				
				pitchCurrent = pitchPosition;
				yawCurrent = yawPosition;
				ROS_INFO_STREAM("\n did case 2");
				break;
			}
			case 3:{ ROS_INFO_STREAM("\n Starting Case 3");
				//get and set the torque commmands into Casey's controller
				double pitchTorque = mostRecentCommand.effort[1];
				double yawTorque = mostRecentCommand.effort[2];
			
				CaseyController.setEffort(1,pitchTorque);
				CaseyController.setEffort(2,yawTorque);
				ROS_INFO_STREAM("\n did case 3");
				break;
			}
			default: //No ROBOT_STATE sent! 
				ROS_WARN("INVALID ROBOT STATE SENT");
				
		}		
		
		//We are still running the current command! 
		newCommandSent = false;
		ROS_INFO_STREAM("\n reached end of ros::ok loop \n \n \n");
		}

	//If ros ends, kill Casey's controller
	CaseyController.~CaseyControl();

}
