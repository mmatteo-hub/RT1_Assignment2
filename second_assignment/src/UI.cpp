#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"
#include "second_assignment/Vel.h"
#include "stdio.h"

// defining a publisher and a client
ros::Publisher pub;
ros::ServiceClient client;

// function to show the menù and give the inout
void callBack()
{
	// defining a variable s_srv of type second_assignment::Service
	second_assignment::Service s_srv;
	// defininf a variable m of type second_assignment::Vel
	second_assignment::Vel m;	
	
	// defining a char to use to store the input
	char inputUsr;
	
	// show the menù
	std::cout << "\n###################### MENU' ######################";
	std::cout << "\nPress:\nw (or W) to accelerate;\ns (or S) to decelerate;\nr (or R) to reset the position;\nq (or Q) to stop the program execution.\n";
	std::cout << "###################################################\n";
	std::cout << "Insert here and press enter to confirm: ";
	// getting the keyboard input
	std::cin >> inputUsr;

	// put the input on the request of the server
	s_srv.request.input = inputUsr;
	// waut for the existance of the server
	client.waitForExistence();
	// call the server
	client.call(s_srv);
	
	// take the ouuput on the response according to the msg defined
	m.m_msg = s_srv.response.output;
	// publish the value obtained
	pub.publish(m);
}

// main
int main(int argc, char ** argv)
{
	// initialising the node
	ros::init(argc, argv, "UI");
	// defining a node handle
	ros::NodeHandle nh;
		
	// advertise the topic /vel
	pub = nh.advertise<second_assignment::Vel>("/vel",1);
	// call the service with the client
	client = nh.serviceClient<second_assignment::Service>("/service");
	
	// spin the prorgram in this way since there is no topic to subscribe to to spin with ros::spin()
	while(ros::ok())
	{
		// call the function to show the menù and take the input at each iteration
		callBack();
		ros::spinOnce();
	}
	
	return 0;
}
