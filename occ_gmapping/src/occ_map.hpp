/******************************************

Please note that I have edited my work for ps6: occupancy grid mapping to model the structure my TA, Victoria Albanese to make it run correctly as it wasn't doing so earlier.


*****************************************/

#ifndef OCC_MAP_HPP
#define OCC_MAP_HPP

#include <vector>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"


//map dimensions
#define MAP_RES 0.1
#define MAP_WIDTH 600
#define MAP_HEIGHT 150
 
//log odds values
#define L_OCCUPIED 1.0
#define L_FREE 0.0
#define L0 0.2
class OccMap {
	private:
		std::vector<std::vector <float> > map;
		ros::Publisher pub_map;
	public:
		OccMap();
		OccMap(ros::NodeHandle nh);
		std::vector<std::vector <float> > get_map();
		void set_map(std::vector<std::vector <float> > new_map);
		
		void publish_map();
		void draw_laser(int x1, int y1, int x, int y, bool is_occ, float laser_range);
		void draw_line(int x1, int y1, int x2, int y2);
		// x1 - robotx, y1 - roboty, x - endptx, y - endpty
		void draw_endpt(bool is_occ, int x, int y);
};

#endif		
