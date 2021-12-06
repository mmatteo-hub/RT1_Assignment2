#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "second_assignment/Service.h"

std_srvs::Empty reset;

float multiplier = 1;

bool setVelocityFnc (second_assignment::Service::Request &req, second_assignment::Service::Response &res)
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
	ros::init(argc, argv, "service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("/service", setVelocityFnc);
	ros::spin();
	
	return 0;
}
