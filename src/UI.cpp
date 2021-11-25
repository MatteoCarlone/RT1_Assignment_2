#include "ros/ros.h"
#include "second_assignment/Accelerate.h"

#define BOLDYELLOW "\033[1m\033[33m"
#define BOLDGREEN "\033[1m\033[32m"
#define BOLDBLUE "\033[1m\033[34m"
#define RESET "\033[0m"

ros::ServiceClient client;

char Input(){
	char input;
	std::cout << BOLDBLUE " Input :  " RESET;
	std::cin >> input;
	return input;
}

void Acceleration(){

}

int main (int argc, char **argv)
{
	
	std::cout << BOLDYELLOW " To accelarate press [a]\n" RESET;
	std::cout << BOLDYELLOW " To decelerate press [d]\n" RESET; 
	std::cout << BOLDYELLOW " To reset the position press [r]\n\n" RESET; 
	
	ros::init(argc, argv, "UI_node");
	ros::NodeHandle n;
	
	client = n.serviceClient<second_assignment::Accelerate>("/accelarate");
	
	second_assignment::Accelerate acc;
	while(ros::ok())
	{
		char input = Input();
		acc.request.input = input;
		
		client.waitForExistence();
		
		client.call(acc);
		
		ROS_INFO(BOLDGREEN " ACCELERATOR: %f\n " RESET ,acc.response.value);
		
		ros::spinOnce();
	}
	return 0;

}
