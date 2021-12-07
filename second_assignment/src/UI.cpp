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
	
	// The menù is not shown by the UI but by the control because they share the same shell
	// since I decided to launch them by .launch file, so to have a clear output and avoid
	// having mistakes in printing the instructions, I decided to put it there and not here.
	// The result does not change, since the UI always take the input and call the service.
	
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
