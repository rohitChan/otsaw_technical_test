// #ifndef NO_ENTRY_LAYER_NODE_H
// #define NO_ENTRY_LAYER_NODE_H

// #define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <no_entry_layer/carto_mask_configConfig.h>
#include <dynamic_reconfigure/server.h>

namespace mapmodifier
{ 	
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
}