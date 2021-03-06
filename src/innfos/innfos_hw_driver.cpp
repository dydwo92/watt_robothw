#include "innfos_hw_driver.h"

bool InnfosHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {

	int i;

	if (!robot_hw_nh.hasParam("DOF")) {
		ROS_ERROR("ROS Parameter DOF not specified!");
		return false;
	}

	/* getParam forbids to change member */
	robot_hw_nh.getParam("DOF", dof_);
	robot_hw_nh.getParam("joint_names", jnt_names_);
	robot_hw_nh.getParam("joint_ids", jnt_ids_);

	if (jnt_names_.size() != dof_) {
		ROS_ERROR("Joint_names array size error!");
		return false;
	}

	if (jnt_ids_.size() != dof_) {
		ROS_ERROR("Joint_ids array size error!");
		return false;
	}

	// Init subscriber
	std::string estop_topic;
	if (!root_nh.getParam("estop_topic", estop_topic)) {
		ROS_ERROR("estop_topic name error!");
		return false;
	}

	std::string eshutdown_topic;
	if (!root_nh.getParam("eshutdown_topic", eshutdown_topic)) {
		ROS_ERROR("eshutdown_topic name error!");
		return false;
	}

	e_stop_active_ = false;
	sub_e_stop_ = root_nh.subscribe(estop_topic, 1, &InnfosHW::callback_activate_e_stop, this);

	e_shutdown_active_ = false;
	sub_e_shutdown_ = root_nh.subscribe(eshutdown_topic, 1, &InnfosHW::callback_activate_e_shutdown, this);

	position_cmd_.resize(dof_);
	last_position_cmd_.resize(dof_);
	position_cmd_float_.resize(dof_);
	position_fdb_.resize(dof_);
	velocity_cmd_.resize(dof_);
	velocity_fdb_.resize(dof_);
	effort_cmd_.resize(dof_);
	effort_fdb_.resize(dof_);

	max_accel_.resize(dof_);
	max_vel_.resize(dof_);

	robot_hw_nh.getParam("max_accel", max_accel_);
	robot_hw_nh.getParam("max_vel", max_vel_);
	if(max_accel_.size() != dof_ || max_vel_.size() != dof_){
		ROS_ERROR("parameter array length error!");
		return false;
	}

	innfos_reply_.resize(dof_);

	for(i = 0; i < dof_; i++){
		INNFOS_Init(jnt_ids_.at(i), max_accel_[i], max_vel_[i]);
	}

	for (i = 0; i < dof_; i++) {
		// Create joint state interface for all joints
		js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[i], &position_fdb_[i], &velocity_fdb_[i], &effort_fdb_[i]));

		hardware_interface::JointHandle joint_handle;
		joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[i]), &position_cmd_[i]);
		pj_interface_.registerHandle(joint_handle);
	}

	registerInterface (&js_interface_);
	registerInterface (&pj_interface_);

	ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());

	return true;
}

void InnfosHW::callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active){
	e_stop_active_ = e_stop_active->data;
}

void InnfosHW::callback_activate_e_shutdown(const std_msgs::BoolConstPtr& e_shutdown_active){

	int i;

	if(e_shutdown_active_ != e_shutdown_active->data){
		for(i = 0; i < dof_; i++){
			if(!e_shutdown_active->data) INNFOS_Init(jnt_ids_.at(i), max_accel_[i], max_vel_[i]);
			else INNFOS_deInit(jnt_ids_.at(i));
		}
		e_shutdown_active_ = e_shutdown_active->data;
	}
}

void InnfosHW::read(const ros::Time& time, const ros::Duration& period){
	// basically the above feedback callback functions have done the job
	int i;
	for(i = 0; i < dof_; i++){
		position_fdb_[i] = innfos_reply_.at(i).Position;
	    velocity_fdb_[i] = innfos_reply_.at(i).Speed;
		effort_fdb_[i] = 0;
	}
}

void InnfosHW::write(const ros::Time& time, const ros::Duration& period){
	int i;

	if(e_shutdown_active_) return;

	for (i = 0; i < NUM_MOTOR; i++) {
		if(!e_stop_active_){
			last_position_cmd_[i] = position_fdb_[i];
			INNFOS_posCmd(&innfos_reply_.at(i), jnt_ids_.at(i), (float)position_cmd_[i], 200);
		}else{
			INNFOS_posCmd(&innfos_reply_.at(i), jnt_ids_.at(i), (float)last_position_cmd_[i], 200);
		}

	}
}

void InnfosHW::deinit(){
	int i;
	for (i = 0; i < dof_; i++) {
		INNFOS_deInit(jnt_ids_.at(i));
	}
}
