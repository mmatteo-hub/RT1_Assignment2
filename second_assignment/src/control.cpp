#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"
#include "second_assignment/Vel.h"

// definition of the size for the right, front, left array
#define SIZE 110

// defining a publisher
ros::Publisher pub;
// defininf a variable vel of type geometry_msgs::Twist
geometry_msgs::Twist vel;

// front threshold for distance
float f_th = 1.5;

// variable to change the velocity
float change_term = 0;
float max_change = 3;
float min_change = 0;

// variable representing the initial value of the velocity for the robot
float start_vel_val = 1;

// defining velocity, linear and angular, while the robot is in some corners
float linear_corner = 0.5;
float angular_corner = 1;

// function to calculate the minimun distance among array values
double min_val(double a[])
{
	// setting dist as the maximux for the laser so that there cannot be errors
	double dist = 30;
	
	// test each element of the array
	for(int i=0; i < SIZE; i++)
	{
		// check if the value is less than the distance
		if(a[i] < dist)
		{
			// update the distance with the value
			dist = a[i];
		}
	}
	// return the minimun distance found
	return dist;
}

// call back function to change the factor for the velocity
void callbackFnc(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	// take the size of the ranges[] array and initialise a new array, r, with the same dimension
	float r[msg->ranges.size()];
	
	// scroll through all the array
	for(int k = 0; k < msg->ranges.size(); k++)
	{	
		// initialising r
		r[k] = msg->ranges[k];
	}

	// defining array representing the front, left and right of the robot
	// the dimension of the array is such that it allows the robot have a good range of view
	double left[SIZE];
	double front[SIZE];
	double right[SIZE];

	// for loops are properly initialised in order to take exactly a range:
	// each for loop is set to scroll SIZE values, but with the initiualization to define right, front and left ranges.

	// left loop
	for(int i = msg->ranges.size()-SIZE; i <= msg->ranges.size()-1; i++)
	{
		left[i-(msg->ranges.size()-SIZE)] = r[i];
	}

	// front loop
	for(int i = (msg->ranges.size()/2 - SIZE/2 - 1); i <= (msg->ranges.size())/2 + SIZE/2 - 1; i++)
	{
		front[i-(msg->ranges.size()/2 - SIZE/2 - 1)] = r[i];
	}

	// right loop
	for(int i = 0; i <= SIZE-1; i++)
	{
		right[i] = r[i];
	}

	// check if the distance is less than a certain threshold
	if(min_val(front) < f_th)
	{
		if(change_term >= min_change && change_term <= max_change)
		{
			// checking if the the right is nearest 
			if(min_val(right) < min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = angular_corner * change_term;
				vel.linear.x = linear_corner * change_term;
			}
			
			// checking if the the left is nearest 
			else if(min_val(right) > min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = - angular_corner * change_term;
				vel.linear.x = linear_corner * change_term;
			}
		}
		
		else if(change_term < min_change)
		{
			// checking if the the right is nearest 
			if(min_val(right) < min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = angular_corner * min_change;
				vel.linear.x = linear_corner * min_change;
			}
			
			// checking if the the left is nearest 
			else if(min_val(right) > min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = - angular_corner * min_change;
				vel.linear.x = linear_corner * min_change;
			}
		}
		else if(change_term > max_change)
		{
			// checking if the the right is nearest 
			if(min_val(right) < min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = angular_corner * max_change;
				vel.linear.x = linear_corner * max_change;
			}
			
			// checking if the the left is nearest 
			else if(min_val(right) > min_val(left))
			{
				// setting the velocity to drive out from a corner
				vel.angular.z = - angular_corner * max_change;
				vel.linear.x = linear_corner * max_change;
			}
		}
	}
	
	// if the robot is far from a frontal wall
	else if(min_val(front) > f_th)
	{
		// control to avoid having velocities too high or too low
		if(change_term >= min_change && change_term <= max_change)
		{
			// setting the velocity according to the service response
			vel.linear.x = start_vel_val * change_term;
			vel.angular.z = 0;
		}
		else if(change_term < min_change)
		{
			// setting the velocity according to the service response
			vel.linear.x = start_vel_val * min_change;
			vel.angular.z = 0;
		}
		
		else if(change_term > max_change)
		{
			// setting the velocity according to the service response
			vel.linear.x = start_vel_val * max_change;
			vel.angular.z = 0;
		}
	}
	system("clear");
	// showing linear and angular velocity
	ROS_INFO("Linear velocity:[%f, %f, %f]\n",vel.linear.x,vel.linear.y,vel.linear.z);
	ROS_INFO("Angular velocity:[%f, %f, %f]\n",vel.angular.x,vel.angular.y,vel.angular.z);
	ROS_INFO("Multiplier:[%f]\n",change_term);
	
	// show the menù
	std::cout << "\n###################### MENU' ######################";
	std::cout << "\nPress:\nw (or W) to accelerate;\ns (or S) to decelerate;\nr (or R) to reset the position;\nq (or Q) to stop the program execution.\n";
	std::cout << "###################################################\n";
	
	// publish the velocity computed
	pub.publish(vel);
}

// function to take the value according to the msg defined in Vel.msg
void callbackFnc2(const second_assignment::Vel::ConstPtr &msg)
{
	change_term = msg->m_msg;
}

// main
int main(int argc, char ** argv)
{
	// initialising the node
	ros::init(argc, argv, "control");
	// defining a node handle
	ros::NodeHandle nh;
	
	// advertise the topic /cmd_vel
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
	
	// defining 2 subscrubers to subscribe to the /vel (message) and /base_scan topics to take infos
	ros::Subscriber sub2 = nh.subscribe("/vel", 1, callbackFnc2);
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, callbackFnc);

	// using this to spin the program
	ros::spin();
	return 0;
}
