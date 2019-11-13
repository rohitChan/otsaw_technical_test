#include <no_entry_layer/no_entry_layer_node.h>

using namespace mapmodifier;

cv::Mat mapmodifier::mapToMat(const nav_msgs::OccupancyGrid *map)
{
    Mat im(map->info.height, map->info.width, CV_8UC1);

    if (map->info.height*map->info.width != map->data.size())
    {
        ROS_ERROR("Data size doesn't match width*height: width = %d, height = %d, data size = %zu", map->info.width, map->info.height, map->data.size());
    }

    // transform the map in the same way the map_saver component does
    for (size_t i=0; i < map->info.height*map->info.width; i++)
    {
        if (map->data.at(i) == 0)
        {
            im.data[i] = 254;
        }
        else
        {
            if(map->data.at(i) == 100)
            {
                im.data[i] = 0;
            }
            else
            {
                im.data[i] = 205;
            }
        }
    }

    return im;
}

nav_msgs::OccupancyGrid* mapmodifier::matToMap(const Mat mat, nav_msgs::OccupancyGrid *forInfo)
{
    nav_msgs::OccupancyGrid* toReturn = forInfo;
    for(size_t i=2; i<toReturn->info.height * toReturn->info.width;i++)
    {
        //toReturn->data.push_back(mat.data[i]);
       if(mat.data[i]== 254)//KNOWN
           toReturn->data[i]=0;
        // here it is <10 and not 0 (like in mapToMat), becouse otherwise we loose much
        //walls etc. but so we get a little of wrong information in the unkown area, what is
        // not so terrible.
        else if(mat.data[i] > -1 && mat.data[i] < 50) toReturn->data[i]=100; //WALL
       else toReturn->data[i] = -1; //UNKOWN
    }
    return toReturn;
}

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
	std::cout << "mask_on: " << mask_on << std::endl;
	std::cout << "cost_value: " << cost_value << std::endl;	
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
	// std::cout << "*new_map_occ_grid.data" << *new_map_occ_grid.data;
	// for (std::vector<int8_t>::const_reverse_iterator iterator = new_map_occ_grid.data.rbegin();
 //      iterator != occupancyGrid.data.rend(); ++iterator) {
 //    size_t i = std::distance(new_map_occ_grid.data.rbegin(), iterator);
 //    data(i) = *iterator != -1 ? *iterator : NAN;
	// }
	// // std::vector<int8_t>::const_reverse_iterator iterator = new_map_occ_grid.data.rbegin();
	// cv::Mat A(map_height, map_width, CV_8S, *iterator);
	// cv::normalize(image, A, 0, 1, cv::NORM_MINMAX);
    // cv::imshow("test", A);
    // cv::waitKey(0);
	
	cv::Mat cv_map = mapmodifier::mapToMat(&new_map_occ_grid); 
    cv::imshow("test", cv_map);
    cv::waitKey(0);	
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
		std::cout << "mask_on. " << std::endl;
		for (int i = 0; i < map_width ; i++)
		{
			for (int j = 0; j < map_height ; j++)
			{
				{
					if ((i > pos_bottomleft_x && j > pos_bottomleft_y)  && (i < pos_topright_x && j < pos_topright_y))
					{
						int a = new_map_occ_grid.data[(j-1)*map_width + i] ;
						
						if (new_map_occ_grid.data[(j-1)*map_width + i]  == 0)
						{
							new_map_occ_grid.data[(j-1)*map_width + i] = cost_value;				
						}
					}

				}
			}
		}			
	}
	/**
	 * Publish map as /new_map
	 */
	masked_map_pub.publish(new_map_occ_grid);
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
	original_map_topic = "/map";
	new_map_topic = "/masked_map";
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
	f = boost::bind(&dynReconfgrCallback, _1, _2);
	server.setCallback(f);
	initialize(n);	
	while(ros::ok())
	{
		/**
		 * If atleast first map is recieved
		 */
		if(map_received)
		{
			processMaps();
		}
		ros::spinOnce();
		r.sleep();		

	}

	return 1;
}