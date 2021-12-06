#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "RT1_Assignment2/Service.h"
#include "RT1_Assignment2/Vel.h"

ros::Publisher pub;
ros::ServiceClient client;
RT1_Assignment2::Service srv; // variable with type Service to publish the input received by the user

void callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	RT1_Assignment2::Service srv;
	RT1_Assignment2::Vel m;	
	
	char inputUsr;
	std::cout << "Press:\nw (or W) to accelerate;\ns (or S) to decelerate;\nr (or R) to reset the position.\n";
	std::cin >> inputUsr;

	srv.request.input = inputUsr;
	client.waitForExistence();
	client.call(srv);
	m.m_msg = srv.response.output;
	pub.publish(m);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "UI_node");
	ros::NodeHandle nh;
		
	pub = nh.advertise<RT1_Assignment2::Vel>("/vel",50);
	ros::ServiceClient client0 = nh.serviceClient<RT1_Assignment2::Service>("/service");
	
	ros::Subscriber sub = nh.subscribe("/base_scan",50,callBack);
	
	ros::spin();
	
	return 0;
}
