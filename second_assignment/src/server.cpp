#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "second_assignment/Service.h"

// defining the value to sum or decrease
#define VAL 0.5

// defining a variable to 
std_srvs::Empty reset;

// variable to pass the velocity change
float change_term = 0;

// switch to choose what to do given a certain input
bool setVelocityFnc (second_assignment::Service::Request &req, second_assignment::Service::Response &res)
{
	switch(req.input)
	{
		// accelerate
		case 'w':
		case 'W':
			change_term += VAL;
			break;
		
		// decelerate
		case 's':
		case 'S':
			change_term -= VAL;
			break;
			
		// reset position
		case 'r':
		case 'R':
			ros::service::call("/reset_positions",reset);
			change_term = 0;
			break;
			
		// kille the node
		case 'q':
		case 'Q':
			ros::shutdown();
			break;
			
		// invalid input
		default:
			std::cout << "Invalid input.";
			break;
	}
	
	// put on the output of the server the value obtained
	res.output = change_term;
	
	return true;
}

// mian
int main(int argc, char ** argv)
{
	// initialising the node
	ros::init(argc, argv, "server");
	// defininf a node handle
	ros::NodeHandle nh;
	// advertise the service and call the function
	ros::ServiceServer service = nh.advertiseService("/service", setVelocityFnc);
	
	// spin the program
	ros::spin();
	
	return 0;
}
