/******************************************

Please note that I have edited my work for ps6: occupancy grid mapping to model the structure my TA, Victoria Albanese to make it run correctly as it wasn't doing so earlier.


*****************************************/

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#define SCALE 10

class Robot{

	private:
		float x_pos;
		float y_pos;
		tf::Quaternion orientation;

		float ls0; //laser sensor 0 at 0*
		float ls1;	//laser sensor 1 at 45*
		float ls2;	// at 90*
		float ls3;	// at 135*
		float ls4;	// at 180*
		
		ros::Publisher pub;
		ros::Subscriber sub_laser;
		ros::Subscriber sub_pose;

	public:
		Robot();
		Robot(ros::NodeHandle nh);
		float get_offset_xpos();
		float get_offset_ypos();
		float get_ls0();
		float get_ls1();
		float get_ls2();
		float get_ls3();
		float get_ls4();
	
		float get_orientation();

		void wall_follow();
		void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser);
		void pose_callback(const nav_msgs::Odometry::ConstPtr& current_pose);

};

#endif 		

