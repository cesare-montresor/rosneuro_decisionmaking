#ifndef ROSNEURO_DYNAMICINTEGRATOR_CPP
#define ROSNEURO_DYNAMICINTEGRATOR_CPP

#include "rosneuro_decisionmaking/DynamicIntegrator.hpp"

namespace rosneuro {

DynamicIntegrator::DynamicIntegrator(void) : p_nh_("~") {

	this->sub_topic_data_ = "/smrbci/neuroprediction";
	this->pub_topic_data_ = "/integrator/neuroprediction";
	this->pub_topic_evt_  = "/events/bus";

	this->is_new_ = false;
	this->x_t_ = 0.5f;
	this->y_t_ = 0.5f;
	this->dt_  = 0.0625f;

}

DynamicIntegrator::~DynamicIntegrator(void) {}

bool DynamicIntegrator::configure(void) {

	float default_phi   = 0.60f;
	float default_omega = 0.30f;
	float default_psi   = 0.10f;
	float default_chi   = 1.00f;

	// Retrieve parameters from server
	ros::param::param<float>("~phi",   this->phi_,   default_phi);
	ros::param::param<float>("~omega", this->omega_, default_omega);
	ros::param::param<float>("~psi",   this->psi_,   default_psi);
	ros::param::param<float>("~chi",   this->chi_,   default_chi);



	// Configure subscribers and publishers
	this->pub_data_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>(this->pub_topic_data_, 1);
	this->pub_evt_  = this->p_nh_.advertise<rosneuro_msgs::NeuroEvent>(this->pub_topic_evt_, 1);
	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1, &DynamicIntegrator::on_receive_data, this);

	// Configure services
	this->srv_reset_ = this->p_nh_.advertiseService("reset", &DynamicIntegrator::on_request_reset, this);
	

	// Dynamic reconfiguration server
	this->reconfig_function_ = boost::bind(&DynamicIntegrator::on_reconfigure_callback, this, _1, _2);
	this->reconfig_server_.setCallback(this->reconfig_function_);



	return true;
}

void DynamicIntegrator::on_receive_data(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {

	// For sake of simplicity, getting only one probability (the integrator is
	// designed only for two classes)
	this->x_t_ = msg->softpredict.data[0];
	this->is_new_ = true;
}

bool DynamicIntegrator::on_request_reset(std_srvs::Empty::Request& req, 
										 std_srvs::Empty::Response& res) {

	ROS_INFO("Reset probabilities requested");
	this->y_t_ = 0.5f;
}

bool DynamicIntegrator::Run(void) {

	float ffree;
	float ffree_max;
	float fbmi;
	float fsys;
	float command = -1;

	if(this->is_new_ == false) {
		return false;
	}

	// Compute Force Free Maximum (for normalization)
	ffree_max = compute_force_free_maximum(this->omega_, this->psi_);
	
	// Compute Force Free
	ffree = compute_force_free(this->y_t_, this->omega_, this->psi_); 	

	// Normalize Force free
	ffree = ffree/ffree_max;

	// Compute Force BMI
	fbmi  = compute_force_bmi(this->x_t_);
	
	// Compute Force Final
	fsys  = compute_force_system(ffree, fbmi, this->chi_, this->phi_);

	// Integrated output
	this->y_t_ = this->y_t_ + this->dt_ * fsys;

	// Publish integrated output
	this->msg_.header.stamp = ros::Time::now();
	this->msg_.softpredict.data = {this->y_t_, 1.0f - this->y_t_};
	this->pub_data_.publish(this->msg_);

	// Publish event command for the control (TO DO: Remove from here)
	if(this->is_command_available(this->y_t_, command) == true) {
		this->evt_.header.stamp = ros::Time::now();
		this->evt_.event		= command;
		this->pub_evt_.publish(this->evt_);
		this->y_t_ = 0.5f;
	}

	// Reset new message flag
	this->is_new_ = false;

	return true;
	
}

bool DynamicIntegrator::is_command_available(float y, float& command) {

	bool ret = true;

	if( y <= 0.0f ) {
		command = BCI_COMMAND[0] + MASK_COMMAND;
	} else if( y >= 1.0f) {
		command = BCI_COMMAND[1] + MASK_COMMAND;
	} else {
		ret = false;
	}

	return ret;
}

float DynamicIntegrator::compute_force_free(float y, float omega, float psi) {

	float ffree = 0.0f;

	if( ( y >= 0.0f ) & ( y < ( 0.5f - omega ) ) ) {
		ffree = -sin( ( ( M_PI/(0.5f - omega) )*y ) );			
	} else if ( ( y >= ( 0.5f - omega ) ) & ( y <= ( 0.5f + omega ) ) ) {
		ffree = -psi * sin ( (M_PI/omega) * ( y - 0.5 ) );
	} else if ( ( y > ( 0.5f + omega ) ) & ( y <= 1.0f ) ) {
		ffree = sin ( ( M_PI/(0.5 - omega) ) * (y - 0.5 - omega ) );
	}

	return ffree;
}

float DynamicIntegrator::compute_force_free_maximum(float omega, float psi) {

	float x    = 0.0f;
	float y	   = 0.0f;
	float step = 0.01f;

	while( x <= 1.0f ) {
		y = std::max(y, std::fabs(compute_force_free(x, omega, psi)));
		x = x + step;
	}
	return y;
}


float DynamicIntegrator::compute_force_bmi(float x) {

	float fbmi;

	fbmi = 6.4f * pow( (x - 0.5f), 3) + 0.4f * (x - 0.5f);

	return fbmi;
}

float DynamicIntegrator::compute_force_system(float ffree, float fbmi, float chi, float phi) {

	float fsys;

	fsys = chi * ( (phi * ffree) + ( (1.0f - phi) * fbmi ) );

	return fsys;
}

void DynamicIntegrator::on_reconfigure_callback(rosneuro_decisionmaking::DynamicIntegratorConfig &config, 
										uint32_t level) {

	// Omega
	if(this->update_if_different(config.omega, this->omega_))
		ROS_WARN("Updated omega to %f", this->omega_);
	
	// Psi
	if(this->update_if_different(config.psi, this->psi_))
		ROS_WARN("Updated psi to %f", this->psi_);
	
	// Chi
	if(this->update_if_different(config.chi, this->chi_))
		ROS_WARN("Updated chi to %f", this->chi_);
	
	// Phi
	if(this->update_if_different(config.phi, this->phi_))
		ROS_WARN("Updated phi to %f", this->phi_);

}

bool DynamicIntegrator::update_if_different(const float& first, float& second, float epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

}

#endif
