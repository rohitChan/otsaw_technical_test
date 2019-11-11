#include <no_entry_layer/no_entry_layer_node.h>


ros::Publisher	masked_map_pub;
nav_msgs::OccupancyGrid new_map_occ_grid;
bool map_received;
bool mask_on;
int cost_value, mask_lb_x, mask_lb_y, mask_rt_x, mask_rt_y;
nav_msgs::OccupancyGrid map_bacup;
void dynReconfgr_callback(no_entry_layer::carto_mask_configConfig &config, uint32_t level) 
{
	mask_on = config.mask_on;
	cost_value = config.cost_value;
	mask_lb_x = config.mask_lb_x;
	mask_lb_y = config.mask_lb_y;
	mask_rt_x = config.mask_rt_x;
	mask_rt_y = config.mask_rt_y;
	std::cout << "mask_on: " << mask_on << std::endl;
	std::cout << "cost_value: " << cost_value << std::endl;	
}


void worldToMap( int &map_x,  int &map_y, double pos_x, double pos_y, nav_msgs::OccupancyGrid map)
{
	map_x = (pos_x - map.info.origin.position.x)/map.info.resolution - 0.5;
	map_y = (pos_y - map.info.origin.position.y)/map.info.resolution - 0.5;
	std::cout << "map_x: " << map_x << std::endl;
	std::cout << "map_y: " << map_y << std::endl;
}

void mapCallback(const nav_msgs::OccupancyGrid &map_input)
{
	std::cout << "map 1" << std::endl;
	map_received = true;
	new_map_occ_grid = map_input;
	map_bacup = map_input;
	std::cout << "map 2" << std::endl;	
}

void process_maps()
{
	new_map_occ_grid = map_bacup; 

	// ros::spinOnce();
	int map_height = new_map_occ_grid.info.height;
	// std::cout << "map_height:: " << map_height << std::endl;
	int map_width = new_map_occ_grid.info.width;
	// std::cout << "map_width:: " << map_width << std::endl;



	int pos_bottomleft_x = map_width/4;
	int pos_bottomleft_y = map_height/4;
	int pos_topright_x = map_width/2;
	int pos_topright_y = map_height/2;
	pos_bottomleft_x = mask_lb_x;
	pos_bottomleft_y = mask_lb_y;
	pos_topright_x = mask_rt_x;
	pos_topright_y = mask_rt_y;

	// int index_init = map_width*(pos_bottomleft_y -1) + pos_bottomleft_x-1;
	// int index_final = map_width*(pos_topright_y -1) + pos_topright_x -1 ;
	// std::cout << "index_init: " << index_init << std::endl;
	// std::cout << "index_final: " << index_final << std::endl;
	
	// std::cout << "cost_value: " << cost_value << std::endl;
	if (mask_on)
	{
		std::cout << "mask_on. " << std::endl;
		for (int i = 0; i < map_width ; i++)
		{
			for (int j = 0; j < map_height ; j++)
			{
				// if (new_map_occ_grid.data[i]  == 0)
				{
					if ((i > pos_bottomleft_x && j > pos_bottomleft_y)  && (i < pos_topright_x && j < pos_topright_y))
					{
						int a = new_map_occ_grid.data[j*map_width + i] ;
						
						if (new_map_occ_grid.data[j*map_width + i]  == 0 )
						{
							new_map_occ_grid.data[j*map_width + i] = cost_value;				
							// std::cout 	<< "new_map_occ_grid.data[j*map_width + i]: " << a << std::endl;	
						}
					}

				}
			}
		}			
	}

	masked_map_pub.publish(new_map_occ_grid);
	// map_received = false;
	std::cout << "out " << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "no_entry_layer_node");
	ros::NodeHandle n;
	ros::Rate r(10);
	map_received = false; 	
	mask_on = false;
	mask_lb_x = 0;
	mask_lb_y = 0;
	mask_rt_x = 0;
	mask_rt_y = 0;
	cost_value = -1;
	masked_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1000);
	ros::Subscriber map_sub = n.subscribe("/map", 1000, mapCallback);	

	dynamic_reconfigure::Server<no_entry_layer::carto_mask_configConfig> server;
	dynamic_reconfigure::Server<no_entry_layer::carto_mask_configConfig>::CallbackType f;
	f = boost::bind(&dynReconfgr_callback, _1, _2);
	server.setCallback(f);
	
	while(ros::ok())
	{
		// if(!map_received)
		// 	std::cout << "waiting for map.." << std::endl;
		// else 
		if(map_received)
		{
			process_maps();
		}
		ros::spinOnce();
		r.sleep();		

	}

	return 1;
}