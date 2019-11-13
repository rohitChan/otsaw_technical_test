
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <no_entry_layer/carto_mask_configConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/GetMap.h>


using namespace cv;

namespace mapmodifier
{ 	
	/**
	 * /static_map service for global costmap
	 */
	ros::ServiceServer service;

	/**
	 * Publiisher for masked map
	 */
	ros::Publisher	masked_map_pub;
	/**
	 * Subscirber for original map
	 */
	ros::Subscriber map_sub; //  
	/**
	 * Map copy holder
	 */
	nav_msgs::OccupancyGrid new_map_occ_grid, map_bacup;
	/**
	 * Orignal and masked map topic for publishing
	 */
	std::string original_map_topic, new_map_topic;
	/**
	 * True if at least one map received
	 */
	bool map_received;
	/**
	 * If mask is desired
	 */
	bool mask_on;
	/**
	 * Masking parameters
	 */
	int cost_value, mask_lb_x, mask_lb_y, mask_rt_x, mask_rt_y;


	void initialize(ros::NodeHandle &);
	void dynReconfgrCallback(no_entry_layer::carto_mask_configConfig &config, uint32_t level) ;
	void mapCallback(const nav_msgs::OccupancyGrid &map_input);
	void processMaps();
	cv::Mat mapToMat(const nav_msgs::OccupancyGrid *map);
	nav_msgs::OccupancyGrid* matToMap(const Mat mat, nav_msgs::OccupancyGrid *forInfo);
}


namespace staticMapService
{
	/**
	 * If map_received, true
	 */
	bool map_received;
	/**
	 * Server for /static_map service
	 */
	ros::ServiceServer service;
	/**
	 * Subscriber for modified map
	 */
	ros::Subscriber map_sub; 
	/**
	 * Occupancy grid to store the map
	 */
	nav_msgs::OccupancyGrid new_map_occ_grid;

	bool sendModifiedMap(nav_msgs::GetMap::Request  &req,
         nav_msgs::GetMap::Response &res);

	void mapCallback(const nav_msgs::OccupancyGrid &map_input);
	void initialize(ros::NodeHandle &n);

}