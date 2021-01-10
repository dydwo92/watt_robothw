#include "zltech_can_driver.h"
#include "CANOpen.h"

#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>

#include <net/if.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>

int s_can; // socketCAN file descriptor

static int can_finish;
static pthread_t p_thread[2];

/********************************************************************/
// CAN Thread functions
/********************************************************************/
void* canopen_checkloop(void* d){

	while(!can_finish){
		CANOpen_timerLoop();
		usleep(1000);
	}

	pthread_exit(NULL);

}

void* canopen_rxloop(void* d){

    struct can_frame frame;

    struct timeval timeout;
    fd_set set;
    int rv;
    int nbytes;

	while(!can_finish){
	    FD_ZERO(&set);
	    FD_SET(s_can, &set);

	    timeout.tv_sec = 0;
	    timeout.tv_usec = 100000;

	    rv = select(s_can + 1, &set, NULL, NULL, &timeout);
	    if(rv > 0){
	        nbytes = read(s_can, &frame, sizeof(frame));

	        // Add INNFOS response
	        CANOpen_addRxBuffer(frame.can_id, frame.data);
	    }
	}

	pthread_exit(NULL);

}

bool CAN_init(const char* can_device){
	// socketCAN variables
	int ret;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_filter rfilter[1];

	can_finish = 0;

	// [[ 1.Create socket ]]
	s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s_can < 0) {
	    ROS_ERROR("socketCAN PF_CAN failed.");
	    return false;
	}

	// [[ 2.Specify can0 device ]]
	strcpy(ifr.ifr_name, can_device);
	ret = ioctl(s_can, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
	    ROS_ERROR("socketCAN ioctl failed.");
	    return false;
	}

	// [[ 3.Bind the socket to can0 ]]
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(s_can, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
	    ROS_ERROR("socketCAN bind failed.");
	    return false;
	}

	// [[ 4.Receive all frame ]]
	rfilter[0].can_id = 0x000;
	rfilter[0].can_mask = 0x000;
	setsockopt(s_can, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	ROS_INFO("%s socketCAN Enabled.", can_device);

	// Create CAN related threads ---------------------------------------->
	int thr_id;

	thr_id = pthread_create(&p_thread[0], NULL, canopen_checkloop, NULL);
	if (thr_id < 0) {
		ROS_ERROR("CAN Check loop thread create error.");
		return 1;
	}

	thr_id = pthread_create(&p_thread[1], NULL, canopen_rxloop, NULL);
	if (thr_id < 0) {
		ROS_ERROR("CAN RX loop thread create error.");
		return 1;
	}

	ROS_INFO("INNFOS CAN Threads established.");

	// Create CAN related threads <----------------------------------------

	return true;
}

void CAN_deinit(){

	can_finish = 1;

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);
}

