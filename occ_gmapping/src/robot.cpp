/******************************************

Please note that I have edited my work for ps6: occupancy grid mapping to model the structure my TA, Victoria Albanese to make it run correctly as it wasn't doing so earlier.


*****************************************/

#include "robot.hpp"



Robot::Robot(){
	this->x_pos = -1.0; //error state
	this->y_pos = -1.0;

	tf::Quaternion new_quat(0,0,0,0);
	this->orientation = new_quat;

	this->ls0 = -1.0;
	this->ls1 = -1.0;
	this->ls2 = -1.0;
	this->ls3 = -1.0;
	this->ls4 = -1.0;
}

Robot::Robot(ros::NodeHandle nh){
	this->x_pos = -1.0; //error state
	this->y_pos = -1.0;

	tf::Quaternion new_quat(0,0,0,0);
	this->orientation = new_quat;

	this->ls0 = -1.0;
	this->ls1 = -1.0;
	this->ls2 = -1.0;
	this->ls3 = -1.0;
	this->ls4 = -1.0;

	this->pub = nh.advertise<geometry_msgs::Twist>("robot/cmd_vel", 10);
	this->sub_laser = nh.subscribe<sensor_msgs::LaserScan>("robot/base_scan", 10, &Robot::laser_callback, this);
	this->sub_pose = nh.subscribe<nav_msgs::Odometry>("stage/base_pose_ground_truth", 10, &Robot::pose_callback, this);
}

float Robot::get_offset_xpos(){
	return (this->x_pos + 28.5)*SCALE; //26.95
}

float Robot::get_offset_ypos(){
	return (this->y_pos + 8.0)*SCALE; //5.3
}

float Robot::get_orientation(){
	return this->orientation.getAngle();
}

float Robot::get_ls0(){
	return this->ls0;
}

float Robot::get_ls1(){
	return this->ls1;
}

float Robot::get_ls2(){
	return this->ls2;
}

float Robot::get_ls3(){
	return this->ls3;
}

float Robot::get_ls4(){
	return this->ls4;
}

void Robot::wall_follow(){
	////////////////// WALL - FOLLOWING //////////////////////////////
	geometry_msgs::Twist move;
	move.linear.x = 1.0;

	double target_right = 4.0;
	double error_right = target_right - this->ls1;

	if(this->ls1 <= 3.0 || this->ls2 <= 3.0){ //check if this works correctly
		move.linear.x = 0.0;
		move.angular.z = error_right + 1.0;
	}
   	else{
		move.angular.z = error_right;
   	}
   	move.linear.x = 1.0;
	this->pub.publish(move);
}

void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser){
	this->ls0 = laser->ranges[0];
	this->ls1 = laser->ranges[1];
	this->ls2 = laser->ranges[2];
	this->ls3 = laser->ranges[3];
	this->ls4 = laser->ranges[4];
}
void Robot::pose_callback(const nav_msgs::Odometry::ConstPtr& current_pose){
	this->x_pos = current_pose->pose.pose.position.x;
	this->y_pos = current_pose->pose.pose.position.y;
	
	tf::Quaternion new_orientation(
		current_pose->pose.pose.orientation.x,
		current_pose->pose.pose.orientation.y,
		current_pose->pose.pose.orientation.z,
		current_pose->pose.pose.orientation.w);
	
	this->orientation = new_orientation;
}




