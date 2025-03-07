/*
 _____ _    _____       _     
|  |  |_|  |   | |___ _| |___ 
|  |  | |  | | | | . | . | -_|
|_____|_|  |_|___|___|___|___|

*/

//////// IMPORTING LIBRARIES ////////

#include "ros/ros.h"
// Header of the custom service /accelerate
#include "second_assignment/Accelerate.h"

/////////////////////////////////////

//// COLORS DEFINITION ////

#define BHWHT "\e[1;97m"
#define BHRED "\e[1;91m"
#define BHYEL "\e[1;93m"
#define BHGRN "\e[1;92m"
#define BHBLUE "\e[1;94m"
#define RESET "\033[0m"

///////////////////////////

// Initializing a ros::ServiceClient object 
// which will be used to call the service later on.

ros::ServiceClient client;

char Input(){

	/*
		Function to get the user's input from the terminal console. 
		This function has no arguments.

		Returns:

			input (char) = the character that the user has digited on the keyboard. 
	*/

	char input;
	std::cout << BHBLUE "   INPUT :  " RESET;
	std::cin >> input;
	std::cout<< "\n" ;
	return input;
}

int main (int argc, char **argv)
{
	
	// Printing on the screen the available commands
	std::cout << BHYEL " To accelarate press: "<< RESET << BHBLUE << "[a]\n" RESET;
	std::cout << BHYEL " To decelerate press: "<< RESET << BHBLUE << "[d]\n" RESET; 
	std::cout << BHYEL " To reset the position press: "<< RESET << BHBLUE << "[r]\n\n" << RESET; 
	
	// Initializing the UI_node.
	ros::init(argc, argv, "UI_node");

	// Setupping the NodeHandle for handling the communications 
	// with the ROS system.
	ros::NodeHandle n;
	
	// Creating the client for the /accelerate service.
	client = n.serviceClient<second_assignment::Accelerate>("/accelarate");
	
	// Instatiating an autogenerated service class,
	// i.e. Declaring a second_assignment::Accelerate object.
	second_assignment::Accelerate acc;

	// Looping since ros is running well.
	// if Everything goes right is an infinite loop.
	while(ros::ok())
	{
		// Calling the function Input() Declared before in the code.
		char input = Input();

		// Assigning the user's input into the request member
		// of the object acc, from the autogenerated class second_assignment::Accelerate 
		acc.request.input = input;
		
		// Wainting for the service 
		client.waitForExistence();

		// Calling the service
		if(client.call(acc)){
		
			if(acc.response.value == -1.000000){
				std::cout<< BHRED " This command does'nt exists \n\n" RESET;
			}
			else{

				ROS_INFO(BHGRN " ACCELERATOR: %f\n " RESET ,acc.response.value);
			}
		}
		else{

			ROS_ERROR("Acceleration failed");

		}
	}
	
	return 0;
}
