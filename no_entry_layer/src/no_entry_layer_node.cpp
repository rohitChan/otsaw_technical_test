#include <no_entry_layer/no_entry_layer_node.h>


/**
 * @brief      Getting input from rqt_reconfigure
 *
 * @param      config  The configuration messages
 * @param[in]  level   The level
 */
void mapmodifier::dynReconfgrCallback(no_entry_layer::carto_mask_configConfig &config, uint32_t level) 
{
	/**
	 * mask_on: true if mask desired
	 * mask_lb_x: x value for left bottom cell
	 * mask_lb_y: y value for left bottom cell
	 * mask_lb_x: x value for top right cell
	 * mask_lb_y: y value for top right cell
	 * cost_value: y cost desired for the region
	 */
	mask_on = config.mask_on;
	cost_value = config.cost_value;
	mask_lb_x = config.mask_lb_x;
	mask_lb_y = config.mask_lb_y;
	mask_rt_x = config.mask_rt_x;
	mask_rt_y = config.mask_rt_y;
}


/**
 * @brief      Receives map from map_server and makes two copies
 *
 * @param[in]  map_input  The map input
 */
void mapmodifier::mapCallback(const nav_msgs::OccupancyGrid &map_input)
{
	map_received = true;
	new_map_occ_grid = map_input;
	map_bacup = map_input;
}

/**
 * @brief      process map received from map server by putting mask defined by rqt_reconfigure
 */
void mapmodifier::processMaps()
{
	/**
	 * copying the unmodified map
	 */
	new_map_occ_grid = map_bacup; 
	/**
	 * getting map info to change the cell values
	 */
	int map_height = new_map_occ_grid.info.height;
	int map_width = new_map_occ_grid.info.width;

	/**
	 * Region identification for making map with input from rqt_reconfigure
	 */
	int pos_bottomleft_x, pos_bottomleft_y, pos_topright_x, pos_topright_y;
	pos_bottomleft_x = mask_lb_x;
	pos_bottomleft_y = mask_lb_y;
	pos_topright_x = mask_rt_x;
	pos_topright_y = mask_rt_y;

	/**
	 * If mask is desired in the given region put cost value in the desired region, only if it is unoccupied
	 */
	if (mask_on)
	{
		for (int i = 0; i < map_width ; i++)
		{
			for (int j = 0; j < map_height ; j++)
			{
				{
					if ((i > pos_bottomleft_x && j > pos_bottomleft_y)  && (i < pos_topright_x && j < pos_topright_y))
					{
						if (new_map_occ_grid.data[(j-1)*map_width + i]  == 0)
						{
							new_map_occ_grid.data[(j-1)*map_width + i] = cost_value;				
						}
					}

				}
			}
		}			
	}
}

/**
 * @brief      Initializes the parameters.
 */
void mapmodifier::initialize(ros::NodeHandle &n)
{
	map_received = false; 	
	mask_on = false;
	mask_lb_x = 0;
	mask_lb_y = 0;
	mask_rt_x = 0;
	mask_rt_y = 0;
	cost_value = -1;
	original_map_topic = "/originalMap/map";
	new_map_topic = "/map";
	masked_map_pub = n.advertise<nav_msgs::OccupancyGrid>(new_map_topic, 1000);
	map_sub = n.subscribe(original_map_topic, 1000, mapCallback);	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "no_entry_layer_node");
	ros::NodeHandle n;
	ros::Rate r(10);
	dynamic_reconfigure::Server<no_entry_layer::carto_mask_configConfig> server;
	dynamic_reconfigure::Server<no_entry_layer::carto_mask_configConfig>::CallbackType f;
	f = boost::bind(&mapmodifier::dynReconfgrCallback, _1, _2);
	server.setCallback(f);
	mapmodifier::initialize(n);	
	while(ros::ok())
	{
		/**
		 * If atleast first map is recieved
		 */
		if (mapmodifier::map_received)
		{
			mapmodifier::processMaps();
			mapmodifier::masked_map_pub.publish(mapmodifier::new_map_occ_grid);			
		}

		ros::spinOnce();
		r.sleep();		

	}

	return 1;
}