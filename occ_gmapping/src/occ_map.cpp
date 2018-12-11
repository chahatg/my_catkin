/******************************************

Please note that I have edited my work for ps6: occupancy grid mapping to model the structure my TA, Victoria Albanese to make it run correctly as it wasn't doing so earlier.


*****************************************/

#include "occ_map.hpp"


OccMap::OccMap(){
	for(int i = 0; i < MAP_HEIGHT; i++){
		std::vector<float> temp;
		for(int j = 0; j < MAP_WIDTH; j++){
			temp.push_back(-1);
		}
		this->map.push_back(temp);
	}
}

OccMap::OccMap(ros::NodeHandle nh){
	for(int i = 0; i < MAP_HEIGHT; i++){
		std::vector<float> temp;
		for(int j = 0; j < MAP_WIDTH; j++){
			temp.push_back(-1);
		}
		this->map.push_back(temp);
	}
	this->pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);
}

void OccMap::publish_map(){
	//Create a header, populate the fields.
	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	header.frame_id = "/world";

	//Create the map meta data
	nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
	metaD.map_load_time = ros::Time::now();
	metaD.resolution = MAP_RES;
	metaD.width = MAP_WIDTH;
	metaD.height = MAP_HEIGHT;
	//metaD.origin will just init to 0, no need to change

	nav_msgs::OccupancyGrid occ_map = nav_msgs::OccupancyGrid();
	occ_map.header = header;
	occ_map.info = metaD;
	for(int i = 0; i < MAP_HEIGHT; i++){

		for (int j = 0; j < MAP_WIDTH; j++){
				float prob = (1.0 - (1.0/ (exp(map[i][j]) + 1.0)))*100;
				occ_map.data.push_back(static_cast<int>(prob)); 
			}
	}
	this->pub_map.publish(occ_map);

}
//, robot.get_west()
void OccMap::draw_laser(int x1, int y1, int x, int y, bool is_occ, float laser_range){
	this->draw_line(x1, y1, x, y);
	if(laser_range != 5.0){
		this->draw_endpt(is_occ, x-1, y-1);
		this->draw_endpt(is_occ, x-1, y);
		this->draw_endpt(is_occ, x, y-1);
		this->draw_endpt(is_occ, x, y);
	}

}

void OccMap::draw_endpt(bool is_occ, int x, int y){

	float current_odds = this->map[y][x];
	if(is_occ){
		current_odds = current_odds + (L_OCCUPIED - L0);
	}
	else{
		current_odds = current_odds + (L_FREE - L0);
	}
	

	if(x > 0 && y > 0){
		if(x < MAP_WIDTH && y < MAP_HEIGHT){
			this->map[y][x] = current_odds;
		}
	}
}
		
void OccMap::draw_line(int x1, int y1, int x2, int y2){
// C program for DDA line generation 
//SOURCE  - https://www.geeksforgeeks.org/dda-line-generation-algorithm-computer-graphics/
  
//Function for finding absolute value 
	// calculate dx & dy 
	int dx = x2 - x1; 
	int dy = y2 - y1;
	  
	// calculate steps required for generating pixels 
	int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy); 
	  
	// calculate increment in x & y for each steps 
	float Xinc = dx / (float) steps; 
	float Yinc = dy / (float) steps; 
	  
	// Put pixel for each step 
	float X = x1; 
	float Y = y1;
	for (int i = 0; i <= steps; i++) { 
		
	   this->draw_endpt(false, X,Y); 
	    X += Xinc;           // increment in x at each step 
	    Y += Yinc;           // increment in y at each step 
	    //delay(100);   
	}
	
}




