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

	// Init CANOpen
	sendPDO_.resize(dof_);
	readPDO_.resize(dof_);
	speed_input_.resize(dof_);
	speed_output_.resize(dof_);
	position_output_.resize(dof_);

	for(i = 0; i < dof_; i++){
	    // RPDO1 Setting
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1400, 0x01, 0x80000200 | jnt_ids_[i], 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1400, 0x02, 0x01, 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1600, 0x00, 0, 1000);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1600, 0x01, 0x60FF0020, 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1600, 0x00, 1, 1000);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1400, 0x01, 0x200 | jnt_ids_[i], 1000);

	    CANOpen_mappingPDO_init(&sendPDO_[i]);
	    CANOpen_mappingPDO_int32(&sendPDO_[i], &speed_input_[i]);

	    // TPDO1 Setting
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1800, 0x01, 0x80000180 | jnt_ids_[i], 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1800, 0x02, 0x01, 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A00, 0x00, 0, 1000);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1A00, 0x01, 0x60640020, 1000);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1A00, 0x02, 0x606C0020, 1000);
	    CANOpen_writeOD_uint8(jnt_ids_[i], 0x1A00, 0x00, 2, 1000);
	    CANOpen_writeOD_uint32(jnt_ids_[i], 0x1800, 0x01, 0x180 | jnt_ids_[i], 1000);

	    CANOpen_mappingPDO_init(&readPDO_[i]);
	    CANOpen_mappingPDO_int32(&readPDO_[i], &position_output_[i]);
	    CANOpen_mappingPDO_int32(&readPDO_[i], &speed_output_[i]);
	}

    sleep(1);
	for (i = 0; i < dof_; i++) {
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0006, 1000);
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0007, 1000);
		CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x000F, 1000);
	}

    CANOpen_NMT(CO_OP);

    joint_cmd_.resize(dof_);
    joint_pos_.resize(dof_, 0);
    joint_vel_.resize(dof_, 0);
    joint_eff_.resize(dof_, 0);

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

void ZltechHW::read(const ros::Time& time, const ros::Duration& period){

	int i;

	// Communicate
    CANOpen_sendSync();
    for(i = 0; i < dof_; i++) CANOpen_readPDO(jnt_ids_[i], 1, &readPDO_[i], 10);

    for(i = 0; i < dof_; i++){
    	joint_eff_[i] = 0;
    	joint_vel_[i] = speed_output_[i] / 10.0 / 60.0 * 2.0 * M_PI;
    	joint_pos_[i] = position_output_[i] / 4096.0 * 2.0 * M_PI;
    }

}

void ZltechHW::write(const ros::Time& time, const ros::Duration& period){

	int i;
	for(i = 0; i < dof_; i++){
		speed_input_[i] = (int32_t)(joint_cmd_[i] * 60.0 / (2.0 * M_PI));
		CANOpen_sendPDO(jnt_ids_[i], 1, &sendPDO_[i]);
	}

}

void ZltechHW::deinit(){

	int i;
	for(i = 0; i < dof_; i++) CANOpen_writeOD_uint16(jnt_ids_[i], 0x6040, 0x00, 0x0000, 0);

}
