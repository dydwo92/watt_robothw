#ifndef WATTROBOTHW_INCLUDE_INNFOS_HW_DRIVER_H_
#define WATTROBOTHW_INCLUDE_INNFOS_HW_DRIVER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "innfos_can_interface.h"

class InnfosHW : public hardware_interface::RobotHW{

public :
	InnfosHW(){};
	~InnfosHW(){};

	bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
	void read(const ros::Time& time, const ros::Duration& period);
	void write(const ros::Time& time, const ros::Duration& period);

	void deinit();

private :
	void callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active);

private :

	// Hardware parameters
	int dof_;
	std::vector<std::string> jnt_names_;
	std::vector<int> jnt_ids_;

	std::vector<struct _INNFOS_REPLY> innfos_reply_;

	// Command variables
	std::vector<double> position_cmd_;
	std::vector<float> position_cmd_float_;
	std::vector<double> velocity_cmd_;
	std::vector<double> effort_cmd_;

	std::vector<double> position_fdb_;
	std::vector<double> velocity_fdb_;
	std::vector<double> effort_fdb_;

	hardware_interface::JointStateInterface    js_interface_;
	hardware_interface::PositionJointInterface pj_interface_;

	ros::Subscriber sub_e_stop_;
	bool e_stop_active_;
};

#endif
