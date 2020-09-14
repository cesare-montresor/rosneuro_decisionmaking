#ifndef INTEGRATOR_CPP
#define INTEGRATOR_CPP

#include "rosneuro_decisionmaking/Integrator.hpp"


namespace rosneuro {

Integrator::Integrator(void) : p_nh_("~") {
	this->sub_topic_data_	= "/smrbci/neuroprediction";
	this->pub_topic_idata_  = "/integrator/neuroprediction"; 
	this->pub_topic_edata_  = "/events/bus"; 

	
}

Integrator::~Integrator(void) {

}


bool Integrator::configure(void) {

	ros::param::param("~rejection_thr",  this->rejection_thr_, 0.55f); 
	ros::param::param("~integration_alpha", this->integration_alpha_, 0.96f);

	ros::param::param("~n_classes", (int&) this->n_classes_, 2);


	// Setup integrator
	// Gloria Beraldo: TO DO generalize with different kind of integrator

	//if(this->integrator_type_.std::string::compare("Exponential")==0) {

		// Setup Integrator
		this->integrator_ = new wtk::proc::Exponential(this->integration_alpha_, this->n_classes_);
	//}


	this->rawpp_ = Eigen::VectorXd::Zero(this->n_classes_);
	this->intpp_ = Eigen::VectorXd::Zero(this->n_classes_); 


	for(int i=0; i<this->n_classes_; i++)
		this->class_labels_.push_back(std::to_string(i+1));
		

	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1000, &Integrator::on_received_data, this);
	this->pub_idata_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>(this->pub_topic_idata_, 1); 
	this->pub_edata_ = this->p_nh_.advertise<rosneuro_msgs::NeuroEvent>(this->pub_topic_edata_, 1); 

	//ros::param::param("~control_thr", this->control_thr_, 0.7f);

	float default_th_value = (float) 1/this->n_classes_;
	for(int i=0; i<this->n_classes_;i++) {
			this->control_default_thr_.push_back(default_th_value);

	}

	ros::param::param<std::vector<float> >("~control_thr",  this->control_thr_, this->control_default_thr_);

	//ROS_DEBUG("Check threshold %f %f", this->control_thr_.at(0), this->control_thr_.at(1));

	this->srv_integrate_   =  this->p_nh_.advertiseService("integrate",  &Integrator::on_request_integrate, this);
	this->srv_reset_   =  this->p_nh_.advertiseService("reset",  &Integrator::on_request_reset, this);


	this->new_raw_prob_ = false;


	this->imsg_.class_labels = this->class_labels_;

	return true;
}

void Integrator::SetRejection(float value) {
	ROS_INFO("Rejection value set at %f", value);
	this->rejection_thr_ = value;
}

void Integrator::SetIntegration(float value) {
	ROS_INFO("Integration value set at %f", value);
	this->integration_alpha_ = value;
	this->integrator_->SetAlpha(this->integration_alpha_);
}

void Integrator::Reset(void) {
	ROS_INFO("Reset Probabilities"); 
	this->integrator_->Reset();
	this->intpp_.fill(0.5);

	//this->imsg_.header.stamp = ros::Time::now();
	//this->imsg_.softpredict.data = std::vector<float>(this->rawpp_.data(), this->rawpp_.data() + this->rawpp_.rows() * this->rawpp_.cols());
	//this->pub_idata_.publish(this->imsg_);
	this->new_command_ = true;

}


void Integrator::on_received_data(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {

	this->imsg_ = *msg;
	this-> new_raw_prob_ = true;

	//this->rawpp_ = Eigen::Map<Eigen::RowVectorXd>(v.data(), v.size());
	
	for(int i=0; i < msg->softpredict.data.size(); i++) {

		this->rawpp_(i) = msg->softpredict.data[i];
	}


}


bool Integrator::Run() {

	if(this->new_raw_prob_== false)
	{
		//ROS_WARN("Not available raw probabilities to integrate");
		return false;
	}

	if(this->rawpp_.maxCoeff(&this->predicted_class_) > this->rejection_thr_) {

		// Apply integration
		this->integrator_->Apply(this->rawpp_, this->intpp_);	
		
	}  

	// Publish the integrated probability
	this->imsg_.header.stamp = ros::Time::now();
	this->imsg_.softpredict.data = std::vector<float>(this->intpp_.data(), this->intpp_.data() + this->intpp_.rows() * this->intpp_.cols());
	this->intpp_.maxCoeff(&this->predicted_class_);
		
	this->hard_prediction_ = std::vector<int> (this->n_classes_);
	this->hard_prediction_.at(this->predicted_class_) = 1;

	this->imsg_.hardpredict.data = this->hard_prediction_;
	this->pub_idata_.publish(this->imsg_);	

	
	// Publish the event notified the new available command
	// Stefano TO DO: move command check to another node
	//if(this->intpp_(this->predicted_class_) > this->control_thr_ && this->new_command_ == true) {
	if(this->intpp_(this->predicted_class_) > this->control_thr_.at(this->predicted_class_) && this->new_command_ == true) {
	ROS_INFO("New command");
	 this->emsg_.header = this->imsg_.header;
	 this->emsg_.header.stamp = ros::Time::now();
	 this->emsg_.event = BCI_COMMANDS[this->predicted_class_] + COMMAND;
	
	 this->pub_edata_.publish(this->emsg_);
	
	 this->new_command_ = false;

	}

	
	this-> new_raw_prob_ = false;

}


bool Integrator::on_request_integrate(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res) {
	return Integrator::Run();
}

bool Integrator::on_request_reset(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res) {
	Integrator::Reset();
	return true;
}


}


#endif
