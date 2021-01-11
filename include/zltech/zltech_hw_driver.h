#ifndef WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_
#define WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "CANOpen.h"

class ZltechHW : public hardware_interface::RobotHW{

public :
	ZltechHW(){};
	~ZltechHW(){};

	bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
	void read(const ros::Time& time, const ros::Duration& period);
	void write(const ros::Time& time, const ros::Duration& period);

	void deinit();

private :

	// Hardware parameters
	int dof_;
	std::vector<std::string> jnt_names_;
	std::vector<int> jnt_ids_;

    std::vector<CO_PDOStruct> sendPDO_;
    std::vector<CO_PDOStruct> readPDO1_;
    std::vector<CO_PDOStruct> readPDO2_;

    std::vector<int32_t> speed_input_;
    std::vector<int32_t> speed_output_;
    std::vector<int32_t> position_output_;
    std::vector<uint16_t> state_output_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;

    std::vector<double> joint_cmd_;
    std::vector<double> joint_pos_;
    std::vector<double> joint_vel_;
    std::vector<double> joint_eff_;

};

#endif /* WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_ */
