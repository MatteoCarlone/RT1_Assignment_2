/*                                                             
 _____         _           _ _            _____       _     
|     |___ ___| |_ ___ ___| | |___ ___   |   | |___ _| |___ 
|   --| . |   |  _|  _| . | | | -_|  _|  | | | | . | . | -_|
|_____|___|_|_|_| |_| |___|_|_|___|_|    |_|___|___|___|___|

*/

//////// IMPORTING LIBRARIES ////////

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Accelerate.h"
#include "std_srvs/Empty.h"

////////////////////////////////////

//// COLORS DEFINITION ////

#define BHWHT "\e[1;97m"
#define BHBLUE "\e[1;94m"
#define RESET "\033[0m"

///////////////////////////

// The Laser of the Robot returns an array of 721 elements,
// here the definition of the size.

#define ARRAY_SIZE 721

// Initializing ros::Publisher object 
// which will be used to published the velocity of the robot.

ros::Publisher pub;

// Initializing a std_srvs::Empty object
// in order to call the standard service /reset_positions

std_srvs::Empty Reset;

//// FUNCTIONS DEFINITION ////

float Wall_Detection(int range_min , int range_max , float laser_array[]);

void Rotation_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);

bool Server(second_assignment::Accelerate::Request &req, second_assignment::Accelerate::Response &res);

geometry_msgs::Twist reaccelerate(geometry_msgs::Twist my_vel);

//////////////////////////////

// xlrate: variable that estimates the degree of acceleration of the robot .

float xlrate = 1.0;

// state: boolean variable to understand whether the robot has turned right after or not.
bool state = true;

// sum: variable used to gradually increase the robot velocity after a curve.
float sum = 0.5;

int main(int argc , char **argv){
	
	// Initializing the Controller_node.
	ros::init(argc ,argv, "carcontroller_node");

	// Setupping the NodeHandle for handling the communications 
	// with the ROS system.
	ros::NodeHandle n;

	// Setting up the publisher for the topic /cmd_vel.
	pub = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1000); 
	
	// Defining the subscriber to the topic /base_scan
	ros::Subscriber sub = n.subscribe("/base_scan", 1000,Rotation_Callback); 

	// Defining the service providing its name "/accelerate"
	// and the callback "Server".
	ros::ServiceServer service =  n.advertiseService("/accelarate", Server); 

	// Function to loop e call the Callback functions.
	ros::spin();

	return 0;

}


float Wall_Detection(int range_min , int range_max , float laser_array[]){

	/*
		Function to detect the minimum distance between the robot and the wall in any angle of its field of view
		i.e. in any angle between 0 and 180 degrees.

		Returns:

			low_value (float) = distance between the robot and the wall. 
	*/

	float low_value = 10.0;
	

	for( int i = range_min ; i < range_max ; i++ ){
				
		if(laser_array[i] <= low_value ){ low_value = laser_array[i]; }	
	}
				
	return low_value;	
				
}

void Rotation_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	/*
		The callback function that subscribes to the topic /base_scan . 
		Here happens the main control of the robot's movement.

		This function has no returns but it publishes the velocity of the robot either linear or angular.

	*/
		
	// Initializing a geometry_msgs::Twist object that contains the linear 
	// and the angular velocity of the robot.

	geometry_msgs::Twist my_vel;

	// Initializing an array that will contain all the robot's laser returns.
	float Laser_Array[ARRAY_SIZE];
	
	// Itializing float variables for the distances between the robot and the wall.
	float dist_right, dist_left , dist_front;
	
	// Filling the Laser array with the datas coming from the topic /base_scan.
	for(int i = 0; i < ARRAY_SIZE ; i++ ){ Laser_Array[i] = msg->ranges[i]; }

	// Using the Wall_Detection function to detect wall on the robot's right, left and front.
	dist_right = Wall_Detection(0,100,Laser_Array);
	dist_left = Wall_Detection(620,720,Laser_Array);
	dist_front = Wall_Detection(290,430,Laser_Array);
	
	/// MOVEMENT ///

	// Checking if there's a wall in front of the robot.
	if ( dist_front < 1.5 ){

		// setting state to false: the robot is going to turn.
		state = false;
	
		// Rotation 
		if ( dist_left > dist_right ){
			my_vel.linear.x = 0.2;
			my_vel.angular.z = 2.0;
		}
	
		else if ( dist_right > dist_left ){ 
			my_vel.linear.x = 0.2;
			my_vel.angular.z = -2.0;
		}
	}
	else{

		// there's no wall in front of the robot.
		switch(state){

			// The robot hasn't just turned 
			case true:

				my_vel.linear.x = 2.0 * xlrate;
				my_vel.angular.z = 0.0;

			break;

			// The robot has just turned and now is speeding up thanks to the reaccelerate function.
			case false:

				my_vel = reaccelerate(my_vel);

			break;
		}	
	}

	std::cout<< BHBLUE  << "  VELOCITY: " << RESET << BHWHT << my_vel.linear.x << RESET << std::endl;

	fflush(stdout);

	// publishing the linear and angular velocity of the robot 

	pub.publish(my_vel);
}

bool Server(second_assignment::Accelerate::Request &req , second_assignment::Accelerate::Response &res ){
	
	/*
		That is the actual server of the project, this function handle the input of the user aquired in the UI_Node.
		if the user press [a], the gloabal variable xlrate is increased.
		if the user press [d], the gloabal variable xlrate is decreased.
		if the user press [r], the service /reset_position is called, and the velocity is restored to its default value.
		if the user press a wrong key a negative response is sent to the UI_Node that will print an error.
	*/

	switch(req.input){
		case 'a':
			xlrate += 0.5;
			res.value = xlrate;
			
		break;
		case 'd':
			xlrate -= 0.5;
			res.value = xlrate;
			
		break;
		case 'r':
			ros::service::call("/reset_positions", Reset);
			xlrate = 1.0;
			res.value = xlrate;
			
		break;
		
		default:
			res.value = -1.0;
		break;
	}
	
	return true;
}


geometry_msgs::Twist reaccelerate(geometry_msgs::Twist my_vel){

	/*
		Function to gradually increase the velocity of the robot after a curve
		
		returns:

			my_vel: a geometry_msgs::Twist object, the increasing velocity of the robot.
	*/

	if(sum >= (xlrate/2)){
		// changing the state variable when the robot has achived an appropriate velocity.s
		state = true;
		sum = 0.5;													
	}
	else{
		my_vel.angular.z = 0.0;
		my_vel.linear.x = 1.0 * sum;
		sum += 0.1;						
	}

	return my_vel;

}

