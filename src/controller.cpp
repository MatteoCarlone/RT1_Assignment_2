#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Accelerate.h"
#include "std_srvs/Empty.h"

#define BOLDRED "\033[1m\033[31m"
#define BOLDBLUE "\033[1m\033[34m"
#define RESET "\033[0m"

#define ARRAY_SIZE 721

ros::Publisher pub;


std_srvs::Empty Reset;

float Wall_Detection(int range_min , int range_max , float laser_array[]);

void Rotation_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);

bool Server(second_assignment::Accelerate::Request &req, second_assignment::Accelerate::Response &res);

float xlrate = 1.0;

int main(int argc , char **argv){
	
	ros::init(argc ,argv, "carcontroller_node");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1000); 
	
	ros::Subscriber sub = n.subscribe("/base_scan", 1000,Rotation_Callback); 
	ros::ServiceServer service =  n.advertiseService("/accelarate", Server); 
	ros::spin();

}


float Wall_Detection(int range_min , int range_max , float laser_array[]){

	float low_value = 10.0;
			
	for( int i = range_min ; i < range_max ; i++ ){
				
		if(laser_array[i] <= low_value ){ low_value = laser_array[i]; }	
	}
				
	return low_value;	
				
}

void Rotation_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	ROS_INFO(BOLDBLUE " Subrscriber \n" RESET);
	fflush(stdout);
		
	geometry_msgs::Twist my_vel;

	float Laser_Array[ARRAY_SIZE];
	
	float dist_right, dist_left , dist_front;
		
	for(int i = 0; i < ARRAY_SIZE ; i++ ){ Laser_Array[i] = msg->ranges[i]; }
	
	dist_right = Wall_Detection(0,100,Laser_Array);
	
	dist_left = Wall_Detection(620,720,Laser_Array);
	
	dist_front = Wall_Detection(310,410,Laser_Array);
	
	if ( dist_front < 1.5 ){
	
		if ( dist_left > dist_right ){
			my_vel.linear.x = 0.2;
			my_vel.angular.z = 1.0;
		}
	
		else if ( dist_right > dist_left ){ 
			my_vel.linear.x = 0.2;
			my_vel.angular.z = -1.0;
		}
	}
	else{
		my_vel.linear.x = 2.0 * xlrate;
		my_vel.angular.z = 0.0;
	}
	
	pub.publish(my_vel);
}

bool Server(second_assignment::Accelerate::Request &req , second_assignment::Accelerate::Response &res ){
	
	switch(req.input){
		case 'a':
			xlrate += 0.5;
			
		break;
		case 'd':
			xlrate -= 0.5;
			
		break;
		case 'r':
			ros::service::call("/reset_positions", Reset);
			
		break;
		
		default:
			std::cout<< BOLDRED" This command does'nt exists \n"RESET;
			return false;
		break;
	}
	
	res.value = xlrate;
	
	return true;
}




