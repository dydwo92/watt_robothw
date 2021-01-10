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

	position_cmd_.resize(dof_);
	position_cmd_float_.resize(dof_);
	position_fdb_.resize(dof_);
	velocity_cmd_.resize(dof_);
	velocity_fdb_.resize(dof_);
	effort_cmd_.resize(dof_);
	effort_fdb_.resize(dof_);

	innfos_reply_.resize(dof_);

	for(i = 0; i < dof_; i++){
		INNFOS_Init(jnt_ids_.at(i));
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
	for (i = 0; i < NUM_MOTOR; i++) {
		INNFOS_posCmd(&innfos_reply_.at(i), jnt_ids_.at(i), (float)position_cmd_[i], 200);
	}
}

void InnfosHW::deinit(){
	int i;
	for (i = 0; i < dof_; i++) {
		INNFOS_deInit(jnt_ids_.at(i));
	}
}
