/*
 * SimulationController.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_

#include "NeuroControllerDriver.h"
#include "ArloDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <string>
#include <iostream>

#define AVANZAR 1
#define GIRAR 2

using namespace std;


class SimulationController {
public:
	SimulationController(ArloDriver* driver);
	virtual ~SimulationController();
	void startSimulation();
	void checkModelPosition(const nav_msgs::Odometry::ConstPtr& msg);
	void checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr& msg);

private:
	ArloDriver* driver;
	double maxSimTime;  /* Maximum time allowed for the robot to get the goal */
	string inputFile;
	string outputFile;
	ros::NodeHandle nh_;
	double linear_;   /* Linear velocity to send to the robot */
	double angular_;  /* Angular velocity to send to the robot */
	double l_scale_;  /* Factor to the scale the linear velocity value */
	double a_scale_;  /* Factor to the scale the angular velocity value */
	ros::Publisher vel_pub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber clock_sub_;
	ros::Subscriber sonar_sub_;
	bool maxTime = false;
	bool finishLineCrossed = false;
	vector<double> sensorValues;
	vector<double> actuatorValues;
};

#endif /* SRC_SIMULATIONCONTROLLER_H_ */
