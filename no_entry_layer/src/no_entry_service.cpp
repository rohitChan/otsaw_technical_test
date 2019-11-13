#include <no_entry_layer/no_entry_layer_node.h>


/**
 * @brief      Sends a modified map for /static_map request.
 *
 * @param      req   The request
 * @param      res   The response map
 *
 * @return     True if service call is successful
 */
bool staticMapService::sendModifiedMap(nav_msgs::GetMap::Request  &req,
         nav_msgs::GetMap::Response &res)
{		
	if(map_received)
	{
		res.map = new_map_occ_grid;
		return true;
	}
	else
		return false;
}



/**
 * @brief      Receives map from map_server and makes two copies
 *
 * @param[in]  map_input  The map input
 */
void staticMapService::mapCallback(const nav_msgs::OccupancyGrid &map_input)
{
	map_received = true;
	new_map_occ_grid = map_input;
}


/**
 * @brief      Initializes the given nodehandle n.
 *
 * @param      n     { parameter_description }
 */
void staticMapService::initialize(ros::NodeHandle &n)
{
	map_received = false; 	
	service = n.advertiseService("/static_map", staticMapService::sendModifiedMap);
	map_sub = n.subscribe("/map", 1000, staticMapService::mapCallback);	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "no_entry_layer_service");
	ros::NodeHandle n;

	ros::Rate r(10);
	staticMapService::initialize(n);	
	while(ros::ok())
	{
		/**
		 * If atleast first map is recieved
		 */
		ros::spinOnce();
		r.sleep();		

	}

	return 1;
}