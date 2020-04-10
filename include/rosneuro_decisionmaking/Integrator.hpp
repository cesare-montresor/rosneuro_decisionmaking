#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroEvent.h" 
#include <wtkprocessing/Exponential.hpp>




///////////////////Gloria Beraldo:  TO DO REMOVE AND GENERALIZE //////////////////////////

int BCI_COMMANDS[2] = {773,771};
int COMMAND = 899;

///////////////////////////////////////////////////////


namespace rosneuro {

class Integrator {
	public:
		Integrator(void);
		virtual ~Integrator(void);
		bool configure(void);

		void SetRejection(float);
		
		void SetIntegration(float);

		void Reset(void);

		bool Run();


       	private:
		void on_received_data(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);

		bool on_request_integrate(std_srvs::Empty::Request& req,
							   std_srvs::Empty::Response& res);
		bool on_request_reset(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);


		void SimulateData(void);	// TO REMOVE

	private:
		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		ros::Subscriber		   sub_data_;
		ros::Publisher		   pub_idata_;
		ros::Publisher		   pub_edata_;

		std::string                sub_topic_data_;
		std::string	           pub_topic_idata_;
		std::string	           pub_topic_edata_;

		std::string                integrator_type_;
		bool 			   new_raw_prob_;
		bool                       new_command_;
		float 			   control_thr_; 
	
		rosneuro_msgs::NeuroOutput imsg_; 
		rosneuro_msgs::NeuroEvent  emsg_;

		
		ros::ServiceServer	srv_integrate_;
		ros::ServiceServer	srv_reset_;

		
		float 		rejection_thr_;
		float 		integration_thr_;
		unsigned int	n_classes_;
		unsigned int    predicted_class_;
		std::vector<int> hard_prediction_;

		std::vector<std::string> class_labels_;	

		Eigen::VectorXd rawpp_;
		Eigen::VectorXd intpp_;

		wtk::proc::Exponential*	integrator_; 

	
		
		
		/////////////////////////////////////////////////////////////////	
		//Gloria Beraldo:
		// TO DO REMOVE JUST TO TEST

		int direction_ = 1;
		float increment_ = 0.001f;
		//////////////////////////////////////////////////////////////



};

}


#endif
