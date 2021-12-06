#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"
#include "second_assignment/Vel.h"

ros::Publisher pub;
geometry_msgs::Twist vel;

// front threshold for distance
float f_th = 1.5;
float f_m_th = 10;
float multiplier = 1;
float output = 0;
float start_vel_val = 1;

float linear_corner = 0.5;
float angular_corner = 1;

float min_lin_vel = 0.25;
float max_lin_vel = 50;

double min_val(double a[])
{
	double dist = 30;
	for(int i=0; i < 100; i++)
	{
		if(a[i] < dist)
		{
			dist = a[i];
		}
	}
	return dist;
}

void callbackFnc(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	// keep array

	float r[msg->ranges.size()];
	for(int k = 0; k < msg->ranges.size(); k++)
	{
		r[k] = msg->ranges[k];
	}

	double left[110];
	double front[110];
	double right[110];

	for(int i = 609; i <= 719; i++)
	{
		left[i-609] = r[i];
	}

	for(int i = 304; i <= 414; i++)
	{
		front[i-304] = r[i];
	}

	for(int i = 0; i <= 109; i++)
	{
		right[i] = r[i];
	}

	// check the distance
	if(min_val(front) < f_th)
	{
		if(min_val(right) < min_val(left))
		{
			vel.angular.z = angular_corner;
			vel.linear.x = linear_corner;
		}
		else if(min_val(right) > min_val(left))
		{
			vel.angular.z = - angular_corner;
			vel.linear.x = linear_corner;
		}
	}
	else if(min_val(front) > f_th)
	{
		// control to avoid having velocities too high or too low
		if(vel.linear.x > min_lin_vel || vel.linear.x < max_lin_vel)
		{
			vel.linear.x = start_vel_val * multiplier;
			vel.angular.z = 0;
		}
	}
	pub.publish(vel);
}

void callbackFnc2(const second_assignment::Vel::ConstPtr &msg)
{
	multiplier = msg->m_msg;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, callbackFnc);
	ros::Subscriber sub2 = nh.subscribe("/vel", 1, callbackFnc2)
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);	

	ros::spin();
	return 0;
}
