#include "zltech_hw_driver.h"

bool ZltechHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh){

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
	sub_e_stop_ = root_nh.subscribe(estop_topic, 1, &ZltechHW::callback_activate_e_stop, this);

	e_shutdown_active_ = false;
	sub_e_shutdown_ = root_nh.subscribe(eshutdown_topic, 1, &ZltechHW::callback_activate_e_shutdown, this);

	// Init CANOpen
	sendPDO_.resize(dof_);
	readPDO1_.resize(dof_);
	readPDO2_.resize(dof_);
	speed_input_.resize(dof_);
	speed_output_.resize(dof_);
	position_output_.resize(dof_);

	activate();

    joint_cmd_.resize(dof_);
    joint_pos_.resize(dof_, 0);
    joint_vel_.resize(dof_, 0);
    joint_eff_.resize(dof_, 0);

	pos_in_coeff_.resize(dof_);
	vel_in_coeff_.resize(dof_);
	vel_out_coeff_.resize(dof_);

	robot_hw_nh.getParam("pos_in_coeff", pos_in_coeff_);
	robot_hw_nh.getParam("vel_in_coeff", vel_in_coeff_);
	robot_hw_nh.getParam("vel_out_coeff", vel_out_coeff_);

	if (pos_in_coeff_.size() != dof_ || vel_in_coeff_.size() != dof_ || vel_out_coeff_.size() != dof_) {
		ROS_ERROR("Coefficient array size error!");
		return false;
	}

	for (i = 0; i < dof_; i++) {
		// Create joint state interface for all joints
		jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]));

		hardware_interface::JointHandle vel_handle_l_wheel(jnt_state_interface_.getHandle(jnt_names_[i]), &joint_cmd_[i]);
		jnt_vel_interface_.registerHandle(vel_handle_l_wheel);
	}

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);

	ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());

	return true;
}

void ZltechHW::activate(){
	int i;

	for(i = 0; i < dof_; i++){
	    // RPDO1 Setting
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1400, 0x01, 0x80000200 | jnt_ids_[i], 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1400, 0x02, 0x01, 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1600, 0x00, 0, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1600, 0x01, 0x60FF0020, 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1600, 0x00, 1, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1400, 0x01, 0x200 | jnt_ids_[i], 100);

	    CANOpen_mappingPDO_init(&sendPDO_[i]);
	    CANOpen_mappingPDO_int32(&sendPDO_[i], &speed_input_[i]);

	    // TPDO1 Setting
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1800, 0x01, 0x80000180 | jnt_ids_[i], 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1800, 0x02, 0x01, 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A00, 0x00, 0, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1A00, 0x01, 0x60640020, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1A00, 0x02, 0x606C0020, 100);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A00, 0x00, 2, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1800, 0x01, 0x180 | jnt_ids_[i], 100);

	    CANOpen_mappingPDO_init(&readPDO1_[i]);
	    CANOpen_mappingPDO_int32(&readPDO1_[i], &position_output_[i]);
	    CANOpen_mappingPDO_int32(&readPDO1_[i], &speed_output_[i]);

		// TPDO2 Setting
		CANOpen_writeOD_uint32(jnt_ids_[i], 0x1801, 0x01, 0x80000180 | jnt_ids_[i], 100);
		CANOpen_writeOD_uint8(jnt_ids_[i], 0x1801, 0x02, 0x01, 100);
		CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A01, 0x00, 0, 100);
		CANOpen_writeOD_uint32(jnt_ids_[i], 0x1A01, 0x01, 0x603F0010, 100);
		CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A00, 0x00, 1, 100);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1801, 0x01, 0x180 | jnt_ids_[i], 100);

		CANOpen_mappingPDO_init(&readPDO2_[i]);
		CANOpen_mappingPDO_uint16(&readPDO2_[i], &state_output_[i]);

	}

    sleep(1);
	for (i = 0; i < dof_; i++) {
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0006, 100);
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0007, 100);
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x000F, 100);
	}

    CANOpen_NMT(CO_OP);
}

void ZltechHW::callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active){
	e_stop_active_ = e_stop_active->data;
}

void ZltechHW::callback_activate_e_shutdown(const std_msgs::BoolConstPtr& e_shutdown_active){

	int i;

	if(e_shutdown_active_ != e_shutdown_active->data){

		if(!e_shutdown_active->data) activate();
		else CANOpen_NMT(CO_RESET);

		e_shutdown_active_ = e_shutdown_active->data;

	}
}

void ZltechHW::read(const ros::Time& time, const ros::Duration& period){

	int i;

	if(e_shutdown_active_) return;

	// Communicate
    CANOpen_sendSync();
    for(i = 0; i < dof_; i++){
		CANOpen_readPDO(jnt_ids_[i], 1, &readPDO1_[i], 10);
		CANOpen_readPDO(jnt_ids_[i], 2, &readPDO2_[i], 10);
	}

    for(i = 0; i < dof_; i++){
    	joint_eff_[i] = 0;
    	joint_vel_[i] = speed_output_[i] / 10.0 / 60.0 * 2.0 * M_PI;
    	joint_pos_[i] = position_output_[i] / 4096.0 * 2.0 * M_PI;

    	joint_vel_[i] *= vel_in_coeff_[i];
    	joint_pos_[i] *= pos_in_coeff_[i];
    }

}

void ZltechHW::write(const ros::Time& time, const ros::Duration& period){

	int i;

	if(e_shutdown_active_) return;

	for(i = 0; i < dof_; i++){
		if (e_stop_active_)
			speed_input_[i] = 0;
		else {
			speed_input_[i] = (int32_t) (joint_cmd_[i] * 60.0 / (2.0 * M_PI));
			speed_input_[i] *= vel_out_coeff_[i];
		}

		CANOpen_sendPDO(jnt_ids_[i], 1, &sendPDO_[i]);
	}

}

void ZltechHW::deinit(){

	int i;
	for(i = 0; i < dof_; i++) CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0000, 0);

}
