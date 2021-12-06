#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "RT1_Assignment2/Service.h"

std_srvs::Empty reset;

float multiplier = 1;

bool setVelocityFnc (RT1_Assignment2::Service::Request &req, RT1_Assignment2::Service::Response &res)
{
	switch(req.input)
	{
		case 'w':
			multiplier *= 1.25;
			break;
		case 's':
			multiplier *= 0.8;
			break;
		case 'r':
			ros::service::call("/reset_position",reset);
			break;
		default:
			std::cout << "Invalid input.";
			break;
	}
	res.output = multiplier;
	
	return true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "service_node");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("/service", setVelocityFnc);
	ros::spin();
	
	return 0;
}
