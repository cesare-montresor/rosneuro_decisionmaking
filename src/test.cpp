#include <ros/ros.h>
#include "rosneuro_decisionmaking/Integrator.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "integrator");

	rosneuro::Integrator integrator;
	
	if(integrator.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(100);
	while(true)
	{

	 if(integrator.Run() == true) {
	   std::cout <<"Integrate" << std::endl; 
	 } 
	
         ros::spinOnce();
	 r.sleep();

	}

	ros::shutdown();
	return 0;
}
