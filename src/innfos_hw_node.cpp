#include "innfos_hw_driver.h"
#include "innfos_can_driver.h"

int main(int argc, char**argv)
{
	ros::init(argc, argv, "innfos_hw");

	ros::NodeHandle nh;
	ros::NodeHandle nh_hw("~");

	// [[ 1. Init socketCAN ]]
	std::string can_device;
	if (nh.getParam("innfos_can_device", can_device)) {
		if (!CAN_init(can_device.c_str()))
			exit(-1);
	} else {
		ROS_ERROR("ROS Parameter can_device not specified!");
	}

	// [[ 2. Init Hardware Interface ]]
	InnfosHW innfos_hw;
	if(!innfos_hw.init(nh, nh_hw)) exit(-1);

	// [[ 3. Get Control rate ]]
	double control_rate;
	if(!nh.getParam("control_rate", control_rate)){
		ROS_ERROR("ROS Parameter control_rate not specified!");
		exit(-1);
	}
	ros::Rate r(control_rate);

	controller_manager::ControllerManager cm(&innfos_hw, nh);

  	ros::AsyncSpinner spinner(4);
	spinner.start();

	// IMPORTANT: DO NOT REMOVE THIS DELAY !!!
	/* Wait for correct initial position to be updated to ros_controller */
	ros::Duration(2.0).sleep();

	ros::Time ts = ros::Time::now();
	while (ros::ok())
	{
	   ros::Duration elapsed = ros::Time::now() - ts;
	   ts = ros::Time::now();
	   innfos_hw.read(ts, elapsed);
	   cm.update(ts, elapsed);
	   innfos_hw.write(ts, elapsed);
	   r.sleep();
	}
	spinner.stop();

	innfos_hw.deinit();
	CAN_deinit();

	return 0;
}
