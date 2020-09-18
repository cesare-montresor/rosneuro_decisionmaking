#include <ros/ros.h>
#include "rosneuro_decisionmaking/DynamicIntegrator.hpp"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "integrator");

	rosneuro::DynamicIntegrator integrator;
	
	if(integrator.configure()== false) {
		ROS_ERROR("Integrator not configured correctly.");
		return -1;
	}

	ros::Rate r(200);

	while(ros::ok()) {

		if(integrator.Run() == true) {} 
	
		ros::spinOnce();
		r.sleep();
	}

	ros::shutdown();
	return 0;
}
