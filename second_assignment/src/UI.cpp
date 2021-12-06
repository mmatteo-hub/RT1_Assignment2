#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"
#include "second_assignment/Vel.h"
#include "stdio.h"

ros::Publisher pub;
ros::ServiceClient client;
second_assignment::Service srv; // variable with type Service to publish the input received by the user

void callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	second_assignment::Service srv;
	second_assignment::Vel m;	
	
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
	ros::init(argc, argv, "UI");
	ros::NodeHandle nh;
		
	pub = nh.advertise<second_assignment::Vel>("/vel",1);
	ros::ServiceClient client0 = nh.serviceClient<second_assignment::Service>("/service");
	
	ros::Subscriber sub = nh.subscribe("/base_scan",50,callBack);
	
	ros::spin();
	
	return 0;
}
