#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include "phantom_omni/OmniFeedback.h"
#include <pthread.h>



struct OmniState {
	double position[3];  //3x1 vector of position
	double velocity[3];  //3x1 vector of velocity
	double inp_vel1[3]; //3x1 history of velocity used for filtering velocity estimate
	double inp_vel2[3];
	double inp_vel3[3];
	double out_vel1[3];
	double out_vel2[3];
	double out_vel3[3];
	double pos_hist1[3]; //3x1 history of position used for 2nd order backward difference estimate of velocity
	double pos_hist2[3];
	double rot[3];
	double joints[3];
	double force[3];   //3 element double vector force[0], force[1], force[2]
	double torque[3];
	float thetas[7];

	bool lock;
	bool torque_control;
	double lock_pos[3];
};

class PhantomROS {

public:
	ros::NodeHandle n;
	ros::Publisher joint_pub;

	ros::Subscriber haptic_sub;
	std::string omni_name;

	OmniState *state;

	void init(OmniState *s) {
		ros::param::param(std::string("~omni_name"), omni_name,
				std::string("omni1"));

                // Publish joint states for robot_state_publisher,
                // and anyone else who wants them.
		std::ostringstream joint_topic;
		joint_topic << omni_name << "_joint_states";
		joint_pub = n.advertise<sensor_msgs::JointState>(joint_topic.str(), 1);

	

		// Subscribe to NAME_force_feedback.
		std::ostringstream force_feedback_topic;
		force_feedback_topic << omni_name << "_force_feedback";
		haptic_sub = n.subscribe(force_feedback_topic.str(), 100,
				&PhantomROS::force_callback, this);

		state = s;
		state->thetas[1] = 0;
		state->thetas[2] = 0;
		state->thetas[3] = 0;
		state->thetas[4] = 0;
		state->thetas[5] = 0;
		state->thetas[6] = 0;
	}

	/*******************************************************************************
	 ROS node callback.
	 *******************************************************************************/
	void force_callback(const phantom_omni::OmniFeedbackConstPtr& omnifeed) {
		state->thetas[1] = -omnifeed->thetas.x;
		state->thetas[2] = omnifeed->thetas.y;
		state->thetas[3] = omnifeed->thetas.z;
		state->thetas[4] = 0;//M_PI;
		state->thetas[5] = 0;//-3*M_PI/4;
		state->thetas[6] = 0;//-M_PI;

		

	}

	void publish_omni_state() {
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);
		joint_state.name[0] = "waist";
		joint_state.position[0] = -state->thetas[1];
		joint_state.name[1] = "shoulder";
		joint_state.position[1] = state->thetas[2];
		joint_state.name[2] = "elbow";
		joint_state.position[2] = state->thetas[3];
		joint_state.name[3] = "wrist1";
		joint_state.position[3] = -state->thetas[4] + M_PI;
		joint_state.name[4] = "wrist2";
		joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
		joint_state.name[5] = "wrist3";
		joint_state.position[5] = -state->thetas[6] - M_PI;
		joint_pub.publish(joint_state);

	}
};


void *ros_publish(void *ptr) {
	PhantomROS *omni_ros = (PhantomROS *) ptr;
	int publish_rate;
	omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
	ros::Rate loop_rate(publish_rate);
	ros::AsyncSpinner spinner(2);
	spinner.start();

	while (ros::ok()) {
		omni_ros->publish_omni_state();
		loop_rate.sleep();
	}
	return NULL;
}

int main(int argc, char** argv) {

	////////////////////////////////////////////////////////////////
	// Init ROS
	////////////////////////////////////////////////////////////////
	ros::init(argc, argv, "omni_haptic_node");
	OmniState state;
	PhantomROS omni_ros;

	omni_ros.init(&state);

	//hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

	////////////////////////////////////////////////////////////////
	// Loop and publish
	////////////////////////////////////////////////////////////////
	pthread_t publish_thread;
	pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
	pthread_join(publish_thread, NULL);

	ROS_INFO("Ending Session....");

	return 0;
}

