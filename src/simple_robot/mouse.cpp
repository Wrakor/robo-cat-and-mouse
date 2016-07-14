#include <iostream>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>      
#include <ros/package.h>
#include <ros/ros.h>
#include <unistd.h>
#include <vector>

#include <stdr_msgs/RobotIndexedVectorMsg.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <turtlesim/Pose.h>

using namespace std;
#define PI 3.14159265358979323846

ros::Publisher cmd_vel_pub_;
geometry_msgs::Twist cmdC;
geometry_msgs::Twist cmdM;
float real_dist = 0;

static double xC = 0, yC = 0, psiC = 0, desiredRotationC = 0;
static double xC1 = 0, yC1 = 0, psiC1 = 0, desiredRotationC1 = 0;
static double xCf = 0, yCf = 0, psiCf = 0, desiredRotationCf = 0;
static double xM = 0, yM = 0, psiM = 0, desiredRotationM = 0;

void catPos(const nav_msgs::Odometry &msg) {
	xC = msg.pose.pose.position.x;
	yC = msg.pose.pose.position.y;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	psiC = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

void cat1Pos(const nav_msgs::Odometry &msg) {
	xC1 = msg.pose.pose.position.x;
	yC1 = msg.pose.pose.position.y;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	psiC1 = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

void mousePos(const nav_msgs::Odometry &msg) {
	xM = msg.pose.pose.position.x;
	yM = msg.pose.pose.position.y;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	psiM = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}


int main(int argc, char ** argv) {
	ros::init(argc, argv, "mouse");
	ros::NodeHandle nh;
	
	// create a publisher object
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1000);

	// create a subscriber object
        ros::Subscriber subCat = nh.subscribe("robot0/odom", 1000, &catPos);
	ros::Subscriber subCat1 = nh.subscribe("robot1/odom", 1000, &cat1Pos);
        ros::Subscriber subMouse = nh.subscribe("robot2/odom", 1000, &mousePos);
   
	// let ROS take over
	ros::spinOnce();
       
    ros::Rate rate(10);
	cmdM.angular.z = 0;
	while(ros::ok) {
		
		float distance = 0;
		float distanceCat = sqrt( pow(xC-xM,2) + pow(yC-yM,2) );
		float distanceCat1 = sqrt( pow(xC1-xM,2) + pow(yC1-yM,2) );
		
		if(distanceCat < distanceCat1) {	
			xCf = xC;
			yCf = yC;
			//ROS_INFO_STREAM("Mouse 1 closer");
			distance = distanceCat;
											
		}
		else {
			xCf = xC1;
			yCf = yC1;
			//ROS_INFO_STREAM("Mouse 2 closer");
			distance = distanceCat1;		
		}
		
		if (distance <= 1.5 && distance != 0) {
			char cmd[50] = "rosrun stdr_robot robot_handler delete /robot2";			
			ROS_INFO_STREAM("morreu");
			system(cmd);
			ros::shutdown();
			while (ros::isShuttingDown() );
		}

		cmdM.angular.z = 0;
		
		/*if( distance < 15) {
		     cmdM.linear.x = 2;
		}
		else {
             cmdM.linear.x = 0;
		}*/
	
       		 // 3 3
		// 3 18
		// 30 18
		// 30 3

		int corner = 0;
		vector<double> dists;		

		double dist1=sqrt( pow(xCf-3,2) + pow(yCf-3,2));
		double dist2=sqrt( pow(xCf-3,2) + pow(yCf-18,2));
		double dist3=sqrt( pow(xCf-30,2) + pow(yCf-18,2));
		double dist4=sqrt( pow(xCf-30,2) + pow(yCf-3,2));

		dists.push_back(dist1);
		dists.push_back(dist2);
		dists.push_back(dist3);
		dists.push_back(dist4);
	
		sort(dists.begin(), dists.end());

		double distCToCenter = sqrt( pow(xCf-15,2) + pow(yCf-9,2));
		double distMToCenter = sqrt( pow(xM-15,2) + pow(yM-9,2));

		ROS_INFO_STREAM("gato " << distCToCenter << " rato" << distMToCenter);
		if (distMToCenter <= distCToCenter) {
			if (dists[3] == dist1)
				corner = 1;
			else if (dists[3] == dist2)
				corner = 2;
			else if (dists[3] == dist3)
				corner = 3;
			else if (dists[3] == dist4)
				corner = 4;

		}
		else {

			double dist = dists[2];
				
			if (dist == dist1)
				corner = 1;
			else if (dist == dist2)
				corner = 2;
			else if (dist == dist3)
				corner = 3;
			else if (dist == dist4)
				corner = 4;

			//ROS_INFO_STREAM("segundo canto mais longe " << corner << " / "<< dists[0] << " " << dists[1] << " " << dists[2] << " " << dists[3]);
		}
		
		
		double angle = 0;

		if (corner == 1)
			angle = atan2(3 - yM, 3 - xM);
		else if (corner == 2)
			angle = atan2(18 - yM, 3 - xM);
		else if (corner == 3)
			angle = atan2(18 - yM, 30 - xM);
		else 
			angle = atan2(3 - yM, 30 - xM);
	
	
		//ROS_INFO_STREAM("angle " << angle);
		if (fabs(psiM - angle) < 0.15)
			cmdM.angular.z = 0;	
		else {
			
			if (  ((psiM - angle < (2*PI - psiM) + angle) && (psiM > angle)) || ((psiM - angle > (2*PI - psiM) + angle) && (psiM < angle)) )

				cmdM.angular.z = -PI/2;
			else
				cmdM.angular.z = PI/2;

			// Walk slower when near walls, so it can turn around
			if( yM >= 17 || yM <= 4 || xM >= 28  || xM <= 4 )
				cmdM.linear.x = 1.5;
			else
				cmdM.linear.x = 2;
		}		
		
	
	    cmd_vel_pub_.publish(cmdM);
		
		cmdM.angular.z = 0;
		ros::spinOnce();
		rate.sleep();	
	}



  
}
