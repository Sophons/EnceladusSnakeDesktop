#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <ncurses.h>
#include <chrono>
#include <cstring>

using namespace std;


// Define publisher data_pub for global use and msg dataTuning
ros::Publisher data_pub;
sensor_msgs::Imu dataTuning;

// Define variables for global use
float oriVar = 1.0;
float linAccVar = 1.0;
float angVelVar = 1.0;
//char press;

void msgfiller(const sensor_msgs::Imu msg){

	//Feed all data from imu/data to dataTuning
	dataTuning = msg;

	//Give new timestamps to stop errors
	dataTuning.header.stamp = ros::Time::now();

	//Replace Covariance matrices
	dataTuning.orientation_covariance[0] = oriVar;
	dataTuning.orientation_covariance[4] = oriVar;
	dataTuning.orientation_covariance[8] = oriVar;

	dataTuning.linear_acceleration_covariance[0] = linAccVar;
	dataTuning.linear_acceleration_covariance[4] = linAccVar;
	dataTuning.linear_acceleration_covariance[8] = linAccVar;

	dataTuning.angular_velocity_covariance[0] = angVelVar;
	dataTuning.angular_velocity_covariance[4] = angVelVar;
	dataTuning.angular_velocity_covariance[8] = angVelVar;

	data_pub.publish(dataTuning);

}

/*
void keyboard(){

  if(press  == ERR){ //no char pressed do nothing
  } else {
        string input;
        cin >> input;
        if(input == "tune"){
                //Accept keyboard inputs to change covariances
                cout << "Enter orientation covariance: ";
                cin >> oriVar;
                cout << "Enter linear acceleration covariance: ";
                cin >> linAccVar;
                cout << "Enter angular velocity covariance: ";
                cin >> angVelVar;
                cout << "Current settings:\n    oriVar= " << oriVar <<
                                         "\n    linAccVar= " << linAccVar <<
                                         "\n    angVelVar= " << angVelVar << "\n";
        } else {
                //nothing
        }
  }
}

*/

int main(int argc, char ** argv) {

    // Initialize ROS node "covarSetter"
    ros::init(argc, argv, "covarSetter");
    ros::NodeHandle n;

    // Get parameters
    std::string key1;
    //double linAccVar = 0.0;
    if (ros::param::search("linAcc_covar", key1))
    {
      ros::param::get(key1, linAccVar);
    }

    std::string key2;
    //double angVelVar = 0.0;
    if (ros::param::search("angVel_covar", key2))
    {
      ros::param::get(key2, angVelVar);
    }

    std::string key3;
    //double oriVar = 0.0;
    if (ros::param::search("ori_covar", key3))
    {
      ros::param::get(key3, oriVar);
    }


    // Listen to "imu/data"
    ros::Subscriber data_sub =
	n.subscribe("/imu/data",1000, msgfiller);

    // Publish on imu/dataTuning
    data_pub =	n.advertise<sensor_msgs::Imu>("dataTuning",1000);
    ros::Rate loop_rate(100);


    //Something to show it's running
    ROS_INFO("Sticking new covariances into dataTuning");

    ros::spin();

    //int ch = getch();
    //nodelay(stdscr, TRUE);
//    while( ros::ok() ){
//	press = getch();
//	keyboard();
//    }
    return 0;

}
