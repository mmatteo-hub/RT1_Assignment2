# Research Track: Assignment #2
## Robot simulator using ROS (Robotics Operative System) and C++.

### Professor [Carmine Recchiuto](https://github.com/CarmineD8), Student: [Matteo Maragliano](https://github.com/mmatteo-hub)

## Running the final code
* First of all it has to be run the `master` node with the command `roscore &` (where `&` allows to run the node in the background and makes the terminal available again to receive inputs).
* Second it has to be run the environment in which the robot will move into, this can be done with `rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world` where `second_assignment` is the package name of the project.
* Later it starts the node with `rosrun second_assignment circuitcontroller_node`;
* then the service with `rosrun second_assignment service_node`;
* at the end the user interface node with `rosrun second_assignment UI_node`.


#### Faster run
Instead of doing all those different things, we provided a particular file `.launch` able to run the program automatically: all what we have to type is `roslaunch second_assignment second.assignment.launch`.
(The file will also run the master node automatically).

## Goal of the assignmnet
The task for this assignment is to make a robot move around a circuit using ROS (Robotics Operative System) and C++. The program should also provide a servie able to set a sort of UI (User Interface) to allow the user inserting different commands for changing the behaviour of the robot.

### General description of the steps followed for the realization
* At the beginning it has been implemented a code able to allow the robot moving inside the environment autonomously: it provides a publisher and a subscriber able to change the behaviour due to the feedback provided by the robot;
* In the second step it has been implemented a user interface to take inputs from keyboard and modify the velocity of the robot inside the circuit. All thoso changes can be computed thanks to a service which establish a communication between all nodes.

## Elements in the project
#### Environment
The environment in which the robot has to move is the *Autodromo Nazionale Monza Formula 1* circuit, properly designed in software, as is can be seen in the following pictures:

<img src="https://user-images.githubusercontent.com/62358773/143142447-3b345203-9e7c-4113-af74-0525f92e6c21.jpg" width=45%, height=45%> <img src="https://user-images.githubusercontent.com/62358773/143142461-1ac1ba12-3d57-43d5-993f-dff0c56135cc.jpg" width=45%, height=45%>

#### Robot

The robot (circled in red in the pictures), is a cube provided with laser scan to detect obstacles, in particular:
* it has 180° of range of vision starting from right (0°) to left (180°);
* it is provided 720 sensors divided in the range of vision: each sensor provides a laser scan acts to detect obstacles and give feedback in a range of 0.25°.

Both the environment and the robot are 3D objects, so their position is determined by a Cartesia tender *(x,y,z)*.

### First step: autonomous controller
In the first step of the project it was created an autonomous controller for the robot.
Since it cannot be known at the beginning the structure of the data we were working on, it was checked by using the shell.
After the environment started it can be checked all nodes running by typing: `rostopic list`; later it has to be found the structure of each node of interesting, for us `/base_scan` which provides data concerning the environment and `/cmd_vel` which provides data about the velocity of the robot. The structure can be taken by: `rostopic info \base_scan` and `rostopic info /cmd_vel`.
A the end of this it has also to be checked the structure of the data by using the command `rosmsg show sensor_msgs/LaserScan` and `rosmsg show geometry_msgs/Twist`.

All data obtained are in the following pictures:

`rostopic list`

<img src="https://user-images.githubusercontent.com/62358773/143204840-4a28a5a8-1f89-45a8-9993-8f0f22165c6b.jpg" width=35%, height=35%>

`rostopic info \base_scan` and `rostopic info /cmd_vel`

<img src="https://user-images.githubusercontent.com/62358773/143204836-6f2c83d2-2031-4f94-b36f-a8218812b5ef.jpg" width=35%, height=35%> <img src="https://user-images.githubusercontent.com/62358773/143204838-5aa377b1-63c2-497b-ba64-e770e5d27da1.jpg" width=35%, height=35%>

`rosmsg show sensor_msgs/LaserScan` and `rosmsg show geometry_msgs/Twist`

<img src="https://user-images.githubusercontent.com/62358773/143204825-a5ab88ae-6d91-4b60-b37e-5a74273f589e.jpg" width=35%, height=35%> <img src="https://user-images.githubusercontent.com/62358773/143204833-6fab1fbe-5abc-49f6-9879-69a5beb14781.jpg" width=35%, height=35%>

It can be clearly seen that the `\base_scan` topic is the Publisher, the one that provides data aquired using the laser in this case, and the structure contains lots of different fields: the most important for us is the `float32[] ranges` in which there are all the 720 sensor values aquired periodically.
On the contrary, the `\cmd_vel` is a Subscriber, the one which computes the velocity, in this case, of the robot and let it move: from the structure it is evidence all fields of the linear and angular velocity among the three axes. Of course, according to the problem we are working on, the linear velocity can be on *x* or *y* and the angular on *z* only; velocity on other axes, in this specific case, do not have any physical reason.


### Second step: controller with interaction by user
In the second step the controller has been implemented by adding the possibility of changing the velocity by inserting inputs from keyboard.
At the beginning the project had only one node, the *circuitcontroller_node* responsible of moving the robot; in this second step we added two more nodes: one for the service, called *service_node* and one for the user interface called *UI_node*.

The general structure is:
* *circuitcontroller_node* : makes the robot move;
* *UI_node* : takes the keyboard inputs;
* *service_node* : reads the value passed by the UI and determines, according to a switch case, the value to pass for changing the velocity.

All those files just described are put inside the *src* folder, as it is usually done. Moreover, for the correct implementation of the program there are still to be defined:
* the type of service;
* (optionally) a type of message to pass the value, nothing more than a format for casting the value passed.

Those two files are put respectively in a *srv* folder and in a *msg* folder.

The service structure given for the implementation is:
* srv:

*char input*

*float32 output*

while the messge structure is
* msg:

*float32 m_msg*

The structure for the communication of the program can be taken by running the graph provided by ROS: the command is `rosrun rqt_graph rqt_graph` and the result is the follwing:

<img src="https://user-images.githubusercontent.com/62358773/145034975-f176a96f-b81c-463d-af7c-49c631ba35c4.jpg" width=75%, height=75%>

The general path to have the velocity changed is:
* UI_node
	* reads the input from keyboard, write the character on the *input* field of the *request* of the service, waits for the existence of the service and then call it;

* server_node
	* switch the *input* field of the request and determines if the requirement is to increase or decrease the velocity: compute the result and then puts it on the *output* field of the response;

* UI_node
	* casts the *output* field of the server response according to the structure of the message *msg* and then publishes the value;

* control_node
	* takes the value published, determines if the velocity can be changed and then publishes the velocity in order to make the robot move faster or slower according to the value inserted.


The server call back function structure is:
```cpp

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
			
		// kill the node
		// inside the .launch file is handled the shoudown of each node
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
```

The control call back function has the due to change the velocity, according to some limitations imposed. Below there is the function in which it is clear the limitation used.
Just before the function it is also shown the group of global variables used:
```cpp
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
}
```
The velocity can be changed both in the straight and in the corner: obviously there is the possibility of a crash if the robot has a velocity too high, in particular inside corners.

#### Criteria for choosing the direction
In order to determine the action the robot has to compute, it has used a simple logic:
* first of all we took all the sensors provied by the `/base_scan` topic and we divided into 3 main categories with the same dimension:
	* right array: from 0th to 109th sensor
	* front array: from 304th to 414th sensor;
	* left array: from 609th to 719th sensor.
* later we compared the distance of the right and on the left and made the robot turn in the direction of the farest distance wall by changing the velocities (inside a turn we use a small linear velocity on *x* direction and a angular velocity on *z* direction) in particular:
	* angular velocity < 0 to turn on the right;
	* angular velocity > 0 to turn on the left.
* at the end if the robot has a straight without any obstacles too near to it it can be changed the velocicty according to the menù provided through the *UI_node*.


### Future Improvements
The robot can be improved for example with the possibility of following the wall, in order to avoid going with a *zig zag* driving in certain situations. Moreover, since not required in the assignment, there could be the possibility of having the best ratio between linear and angular velocity during the corner in order to avoid collisions while turnings.

Except for those improvements the robot has a good behaviour, the velocity cannot be increased too much and with a reasonable multiplier given it can drive all along the circuit without any problems and for many laps: for example with a multiplier = 2 (or 1.5) the robot is able to go along the circuit without any problems; if the multiplier is 3 of 2.5 there is the possibility to have a collision with walls in some points.





