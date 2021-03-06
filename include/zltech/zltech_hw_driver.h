#ifndef WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_
#define WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

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
	void callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active);
	void callback_activate_e_shutdown(const std_msgs::BoolConstPtr& e_shutdown_active);

    void activate();

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

    std::vector<double> pos_in_coeff_;
    std::vector<double> vel_in_coeff_;
    std::vector<double> vel_out_coeff_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;

    std::vector<double> joint_cmd_;
    std::vector<double> joint_pos_;
    std::vector<double> joint_vel_;
    std::vector<double> joint_eff_;

    ros::Subscriber sub_e_stop_;
    bool e_stop_active_;

	ros::Subscriber sub_e_shutdown_;
	bool e_shutdown_active_;
};

#endif /* WATT_ROBOTHW_INCLUDE_ZLTECH_ZLTECH_HW_DRIVER_H_ */
