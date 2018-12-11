/******************************************

Please note that I have edited my work for ps6: occupancy grid mapping to model the structure my TA, Victoria Albanese to make it run correctly as it wasn't doing so earlier.


*****************************************/

#include <cmath>

#include "ros/ros.h"
#include "robot.hpp"
#include "occ_map.hpp"


#define SCALE 10


int main(int argc, char* argv[]){

	ros::init(argc, argv, "mapping");
	ros::NodeHandle nh;

	Robot robot(nh);
	OccMap map(nh);

	ros::Rate loop_rate(10);

	ros::spinOnce();

	
	int bot_x, bot_y, laser_x, laser_y;
	while(ros::ok()){
		//moving the robot
		robot.wall_follow();
		//getting its position
		bot_x = robot.get_offset_xpos();
		bot_y = robot.get_offset_ypos();
		// drawing robot trail
		map.draw_endpt(bot_x, bot_y, false);
		
		//drawing ls0 
		laser_x = robot.get_ls0()*SCALE*cos(robot.get_orientation() - M_PI/2) + bot_x;
		laser_y = robot.get_ls0()*SCALE*sin(robot.get_orientation() - M_PI/2) + bot_y;
		float laser_reading = robot.get_ls0();
		map.draw_laser(bot_x, bot_y, laser_x, laser_y, true, laser_reading);

		//drawing ls1 
		laser_x = robot.get_ls1()*SCALE*cos(robot.get_orientation() - M_PI/4) + bot_x;
		laser_y = robot.get_ls1()*SCALE*sin(robot.get_orientation() - M_PI/4) + bot_y;
		laser_reading = robot.get_ls1();
		map.draw_laser(bot_x, bot_y, laser_x, laser_y, true, laser_reading);		


		//drawing ls2
		laser_x = robot.get_ls2()*SCALE*cos(robot.get_orientation()) + bot_x;
		laser_y = robot.get_ls2()*SCALE*sin(robot.get_orientation()) + bot_y;
		laser_reading = robot.get_ls2();
		map.draw_laser(bot_x, bot_y, laser_x, laser_y, true, laser_reading);

		//drawing ls3
		laser_x = robot.get_ls3()*SCALE*cos(robot.get_orientation() + M_PI/4) + bot_x;
		laser_y = robot.get_ls3()*SCALE*sin(robot.get_orientation() + M_PI/4) + bot_y;
		laser_reading = robot.get_ls3();
		map.draw_laser(bot_x, bot_y, laser_x, laser_y, true, laser_reading);

		//drawing ls4
		laser_x = robot.get_ls4()*SCALE*cos(robot.get_orientation() + M_PI/2) + bot_x;
		laser_y = robot.get_ls4()*SCALE*sin(robot.get_orientation() + M_PI/2) + bot_y;
		laser_reading = robot.get_ls4();
		map.draw_laser(bot_x, bot_y, laser_x, laser_y, true, laser_reading);

		//publish map
		map.publish_map();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}



