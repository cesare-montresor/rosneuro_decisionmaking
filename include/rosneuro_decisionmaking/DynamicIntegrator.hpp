#ifndef ROSNEURO_DYNAMICINTEGRATOR_HPP
#define ROSNEURO_DYNAMICINTEGRATOR_HPP

#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_msgs/NeuroEvent.h"
#include "rosneuro_decisionmaking/DynamicIntegratorConfig.h"


namespace rosneuro {

int BCI_COMMAND[2] = {773, 771};
int MASK_COMMAND   = 899;

typedef dynamic_reconfigure::Server<rosneuro_decisionmaking::DynamicIntegratorConfig> DynamicIntegratorReconfig;

class DynamicIntegrator {

	public:
		DynamicIntegrator(void);
		virtual ~DynamicIntegrator(void);

		bool configure(void);
		bool Run(void);

	private:
		void on_receive_data(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
		bool on_request_reset(std_srvs::Empty::Request& req,
							  std_srvs::Empty::Response& res);
		void on_reconfigure_callback(rosneuro_decisionmaking::DynamicIntegratorConfig &config, 
									 uint32_t level);

		float compute_force_free(float y, float omega, float psi);
		float compute_force_free_maximum(float omega, float psi);
		float compute_force_bmi(float x);
		float compute_force_system(float ffree, float fbmi, float chi, float phi);
		bool  is_command_available(float y, float& command);


		bool  update_if_different(const float& first, float& second, float epsilon = 0.00001f);
		

	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle		p_nh_;
		ros::Subscriber		sub_data_;
		ros::Publisher		pub_data_;
		ros::Publisher		pub_evt_;

		std::string			sub_topic_data_;
		std::string			pub_topic_data_;
		std::string			pub_topic_evt_;

		ros::ServiceServer	srv_reset_;

		DynamicIntegratorReconfig				reconfig_server_;
		DynamicIntegratorReconfig::CallbackType	reconfig_function_;		
		
		rosneuro_msgs::NeuroOutput msg_; 
		rosneuro_msgs::NeuroEvent  evt_; 

		float	x_t_;
		float	y_t_;
		float	dt_;
		bool	is_new_;

		float   omega_;
		float	psi_;
		float   chi_;
		float   phi_;
};

}


#endif
