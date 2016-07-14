// this program randomly-generated velocity messages for turtlesim
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //package_name/type_name
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <turtlesim/Pose.h>
#include <math.h>

#define PI 3.14159265358979323846

static double x = 0, y = 0, psi = 0, desiredRotation = 0;
static double mouse_x = 0, mouse_y = 0, mouse_psi = 0, mouse_vel = 0;
static double mouse2_x = 0, mouse2_y = 0, mouse2_psi = 0, mouse2_vel = 0;
static bool mouse1alive = true, mouse2alive = true;

void poseMessageReceived(const nav_msgs::Odometry &msg) {
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	psi = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

void mousePoseReceived(const nav_msgs::Odometry &msg) {
	mouse_x = msg.pose.pose.position.x;
	mouse_y = msg.pose.pose.position.y;
	mouse_vel = msg.twist.twist.linear.x;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	mouse_psi = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

void mouse2PoseReceived(const nav_msgs::Odometry &msg) {
	mouse2_x = msg.pose.pose.position.x;
	mouse2_y = msg.pose.pose.position.y;
	mouse2_vel = msg.twist.twist.linear.x;
	double q0 = msg.pose.pose.orientation.w;
  	double q1 = msg.pose.pose.orientation.x;
  	double q2 = msg.pose.pose.orientation.y;
  	double q3 = msg.pose.pose.orientation.z;
  	mouse2_psi = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

int main(int argc, char ** argv) {
	// init ROS
	ros::init(argc, argv, "cat");
	ros::NodeHandle nh;

	// create a publisher object
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
	
	// create a subscriber object
	ros::Subscriber cat = nh.subscribe("robot0/odom", 1000, &poseMessageReceived);
	ros::Subscriber mouse = nh.subscribe("robot2/odom", 1000, &mousePoseReceived);
	ros::Subscriber mouse2 = nh.subscribe("robot3/odom", 1000, &mouse2PoseReceived);

	// let ROS take over
	ros::spinOnce();
	
	geometry_msgs::Twist msgRotate, msgWalk;
	msgRotate.linear.x = 1.5;
	msgRotate.angular.z = 0;

	msgWalk.linear.x = 1.5;
	msgWalk.angular.z = 0;

	// loop at 10Hz until the node is shutdown
	ros::Rate rate(10);
	rate.sleep();

	while(ros::ok) {
		// Check which mouse is closer	
		double distanceToMouse = sqrt( pow(mouse_x-x, 2) + pow(mouse_y-y, 2));
		double distanceToMouse2 = sqrt( pow(mouse2_x-x, 2) + pow(mouse2_y-y, 2));
		
		if (distanceToMouse <= 1.5 && distanceToMouse != 0)
			mouse1alive = false;

		if (distanceToMouse2 <= 1.5 && distanceToMouse2 != 0)
			mouse2alive = false;

		double closestMouseX, closestMouseY, closestMousePsi, closestMouseVel;

		if ( (distanceToMouse < distanceToMouse2) || !mouse2alive) {
			closestMouseX = mouse_x;
		    closestMouseY = mouse_y;
		    closestMousePsi = mouse_psi;
		    closestMouseVel = mouse_vel;
		}
		else if ( (distanceToMouse > distanceToMouse2) || !mouse1alive) {
		    closestMouseX = mouse2_x;
		    closestMouseY = mouse2_y;
		    closestMousePsi = mouse2_psi;
		    closestMouseVel = mouse2_vel;
		}
		
		// Preview where mouse is going so we can try to intersect him
		double mouseYPreview = closestMouseY + sin(closestMousePsi)*closestMouseVel;
		double mouseXPreview = closestMouseX + cos(closestMousePsi)*closestMouseVel;	
		double angle = atan2(mouseYPreview - y, mouseXPreview - x);

		if (fabs(psi - angle) < 0.15)
			pub.publish(msgWalk);	
		else {
			angle += PI;
			psi += PI;

			if ( ((psi - angle < (2*PI - psi) + angle) && (psi > angle)) || ((psi - angle > (2*PI - psi) + angle) && (psi < angle)) )
				msgRotate.angular.z = -PI/2;
			else
				msgRotate.angular.z = PI/2;

			// Walk slower when near walls, so it can turn around faster
			if( y >= 17 || y <= 4 || x >= 28  || x <= 4 )
				msgRotate.linear.x = 1;
			else
				msgRotate.linear.x = 1.2;
		
			pub.publish(msgRotate);	
		}	

		// wait until its time for another iteration
		ros::spinOnce();
		rate.sleep();		
	}
}
