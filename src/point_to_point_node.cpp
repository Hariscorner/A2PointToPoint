//Code adapted from UTARI ROS Tutorials
//#include <iomanip>
//#include <iostream>
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"										// to publish velocity to turtle
#include "geometry_msgs/Pose2D.h"										// to get goal position of turtle
#include "turtlesim/Pose.h"												// current turtle position and velocity

bool STOP = true;														
turtlesim::Pose currPose;												// current pose of turtle
geometry_msgs::Pose2D goalPose;											// goal pose of turtle
geometry_msgs::Twist cmdVel;

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg);
void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

float GetXError(turtlesim::Pose currPoseL, geometry_msgs::Pose2D goalPoseL);
float GetThetaError(turtlesim::Pose currPoseL, geometry_msgs::Pose2D goalPoseL);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_to_point_controller"); // connect to roscore
	ros::NodeHandle nh; 
	
	//double x_goal;
	//double y_goal;
	
	//std::cout<<"SETTING GOAL POSITION"<<std::endl;
	//std::cout<<"Please enter the x cordinate (hint: a value between 0.5 to 10.5): ";
	//std::cin>>x_goal;
	//std::cout<<"Please enter the y cordinate (hint: a value between 0.5 to 10.5): ";
	//std::cin>>y_goal;	
	
	//std::cout<<"The goal position is set to ("<< x_goal << "," << y_goal << ")."<<std::endl;

	// subscriber to get current pose of turtle
	ros::Subscriber curr_pose_sub = nh.subscribe("/turtle1/pose", 5, currPoseCallback);
	// subscriber to get goal pose of turtle
	ros::Subscriber goal_pose_sub = nh.subscribe("/turtle1/goalPosition", 5, goalPoseCallback);	//subscribe to goalPosition when it is published by user through the terminal
	// publisher for turtle velocity
	//ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel_reversed", 1000);

	ros::Rate my_rate(10);
	float ErrorX = 0;
	float ErrorTheta = 0;
	
	while (ros::ok() && nh.ok())
	{	
		ros::spinOnce();
		if (STOP == false) 
		{ 
			printf("Processing...\n"); 
			ErrorX		= GetXError(currPose, goalPose);
			ErrorTheta	= GetThetaError(currPose, goalPose);
			
			if (ErrorX >0) 	{ cmdVel.linear.x 	= 0.2 * ErrorX; }
			else 			{	cmdVel.linear.x 	= 0; }
			cmdVel.angular.z 	= 1.0 * ErrorTheta; 
			turtle_vel_pub.publish(cmdVel);
		}
		else 
		{ printf("Waiting...\n"); }
		my_rate.sleep();
	}
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Received curr pose");
	currPose.x 		= msg->x;
	currPose.y 		= msg->y;
	currPose.theta 	= msg->theta;
	return;
}
void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ROS_INFO("Received goal pose");
	STOP				= false;
	goalPose.x			= msg->x;
	goalPose.y			= msg->y;
	goalPose.theta		= msg->theta;
	return;
}

float GetThetaError (turtlesim::Pose currPoseL, geometry_msgs::Pose2D goalPoseL)
{
	float Ex = goalPoseL.x - currPoseL.x;									
	float Ey = goalPoseL.y - currPoseL.y;									
	
	// get desire angle
	float dest = atan2f(Ey, Ex); 										// use float version to get arc tangent
	
	// get angle error
	float Et = dest - currPoseL.theta;
	
	//~ ROS_INFO("Ex: %f, Ey: %f, Et: %f", Ex, Ey, Et);
	return Et;
}
float GetXError(turtlesim::Pose currPoseL, geometry_msgs::Pose2D goalPoseL)
{
	// create error vector
	float Ex = goalPoseL.x - currPoseL.x;									
	float Ey = goalPoseL.y - currPoseL.y;									
	float Et = GetThetaError(currPoseL, goalPoseL);							// get angle between vectors
	
	// project error onto turtle x axis
	//~ float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
	float Etx = hypotf(Ex, Ey)*cos(Et); // improved c function
	
	return Etx;
}