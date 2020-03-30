#include <ros/ros.h>
#include <math.h>
#include <signal.h>
#include <sim_phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

bool interrupted = false;

void mySigintHandler(int sig)
{
  interrupted = true;
}

//Theta
double t1, t2, t3;

//Link's length
double a2 = 0.135;
double a3 = 0.135;

//Link's Moment of Inertia
double Ix = 0.035 + 0.1;
double Iy = 0.035;
double Iz = 0.1;

//Proportional Gain
double Kpx = 15000;
double Kpy = 36000;
double Kpz = 30000;

//Derivative Gain
double Kdx = 500.0;
double Kdy = 2600;
double Kdz = 300;

//cur = Current, An=Angle, Pos = Position, Vel = Velocity, Acc = Acceleration, prev = Previous, Des = Destination
geometry_msgs::Vector3 curAn, curPos, initialPos, finalPos, initialAngle, finalAn, curAnVel, curAnAcc, prevAn, prevAnVel;

geometry_msgs::Vector3 curDesPos, prevDesPos, curDesAn, curDesAnVel, curDesAnAcc, prevDesAn, prevDesAnVel, prevDesAnAcc;

ros::Time prevTime;

float angleValue(float x0, float xf, float t0, float tf, float t)
{
  t = t-t0;
  tf = tf-t0;
  if (t <= tf) {
    return t*(xf-x0)/(tf-t0) + x0; 
  } else {
    return xf;
  }
}

geometry_msgs::Vector3 inverse_kin(geometry_msgs::Vector3 pos)
{
	//Inverse kinematics[!]
	geometry_msgs::Vector3 angles;
	return angles;
}

geometry_msgs::Vector3 forward_kin(geometry_msgs::Vector3 angles)
{
	//Forward kinematics[!]
	geometry_msgs::Vector3 pos
	return pos;
}

void angle_cb(const sensor_msgs::JointState::ConstPtr msg)
{
	//Callback joint_state -- from actual manipulator 
	ros::Time t = ros::Time::now();
	curAn.x = msg->position[0];
	curAn.y = msg->position[1];
	curAn.z = msg->position[2]-(M_PI/2); //!!!
	double diff = t-prevTime.toSec();
	curAnVel.x = (curAn.x - prevAn.x) / diff;
	curAnVel.y = (curAn.y - prevAn.y) / diff;
	curAnVel.z = (curAn.z - prevAn.z) / diff;

	curAnAcc.x = (curAnVel.x - prevAnVel.x) / diff;
	curAnAcc.y = (curAnVel.y - prevAnVel.y) / diff;
	curAnAcc.z = (curAnVel.z - prevAnVel.z) / diff;

	prevAn = curAn;
	prevAnVel = curAnVel;
	prevTime = t;
	curPos = inverse_kin(curAn);
}

phantom_omni::OmniFeedback get_torque(ros::Time time_now, ros::Time time_last)
{
	phantom_omni::OmniFeedback msg;
	msg.thetas = curDesAn;
	msg.thetas.z = (msg.thetas.z+M_PI/2); 
	retrun msg;
}


int main(int argc, char *argv)
{
	ros::init( argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Publisher pub_ = n.advertise<std_msgs::string>("omni1_force_feedback", 1000); 
	prevTime = ros::Time::now();
	ros::Subscriber haptic_sub = n.subscribe("omni1_joint_states", 100, angle_callback);
	ros::Rate loop_rate(1);

	phantom_omni::OmniFeedback msg;
	ros::Time t_0 = ros::Time::now()+ ros::Duration(3.0)
	ros::Time t_f = t_0 + ros::Duration(5.0);

	bool setup = true;

	ros::Time prevTimeLoop =ros::Time::now();
	enum State{STANDBY, JOINTMOVE, POSMOVE, CIRCLE, LINE, FINISH};
	State curState = STANDBY;
	while (ros::ok()&& !interrupted)
	{
		signal(SIGINT, mySigintHandler);
		ros::Time t = ros::Time::now();

		switch (curState){
		case STANDBY:
		
			t_0 = ros::Time::now()+ ros::Duration(30.0);
			t_f = t_0 + ros::Duration(5.0);
			curState = MOVE;
		break;

		case JOINTMOVE:
		
		if(!setup){
			ROS_INFO("Starting Motion - Joint Mode");
			initialPos = curPos;
			initialAn.x = 0;
			initialAn.y = 0;
			initialAn.z = -M_PI/2;
			setup = true;
			t_f = t_0 + ros::Duration(3.0);
			finalAn.x = M_PI/6;
			finalAn.y = M_PI/6;
			finalAn.z = M_PI/6 - M_PI/2;
		}
		if (t > t_0 && t < t_f){
			prevDesAn = curDesAn;
			prevDesPos = curDesPos;
			prevDesAnVel = curDesAnVel;
			prevDesAnAcc = curDesAnAcc

			curDesAn.x = angleValue(initialAn.x,finalAn.x, 0, (t_f-t_0), (t-t_0));
			curDesAn.y = angleValue(initialAn.y,finalAn.y, 0, (t_f-t_0), (t-t_0));
			curDesAn.z = angleValue(initialAn.z,finalAn.z, 0, (t_f-t_0), (t-t_0));

			msg = get_torque(t,prevTimeLoop);

		   }else if(t > t_f){
			curState = FINISH;
			setup = false;
			ROS_INFO("Target reached");
		   }
		break;		

	default:
	
		cout<<"Hello World"<<endl;
	break;
  }  
  pub_.publish(msg);
   
  prevTimeLoop = t;
  ros::spinOnce();
  loop_rate.sleep();

  if(interrupted)
  {
    msg.torque.x = 0;
    msg.torque.y = 0;
    msg.torque.z = 0;
    pub.publish(msg);
    ros::shutdown();
  }

}
  return 0;
}

















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































        \   / 
         .\-/. 
     /\  () ()  /\ 
    /  \ /~-~\ /  \ 
        y  Y  V 
  ,-^-./   |   \,-^-.  Literally a bug
 /    {    |    }    \ 
       \   |   / 
       /\  A  /\ 
      /  \/ \/  \ 
     /           \