#include "ros/ros.h"
#include "robot/GetPoseSRV.h"
#include "robot/SetPoseSRV.h"
#include "robot/SetTrajectorySRV.h"
#include "robot/CommandSRV.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBool.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <iostream>

#include "main.h"
#include "lpp.h"

#define MAP_START_X 			0.154 // 185cm, starting position in MAP_SIZE precentage
#define MAP_START_Y 			0.146 // 175cm, must be same than for hector mapping
//#define MAP_OFFSET_X int(MAP_SIZE*MAP_START_X)
//#define MAP_OFFSET_Y int(MAP_SIZE*MAP_START_Y)
#define MAP_M2PIXEL float(1/MAP_RESOLUTION)
#define ROBOT_CENTER_OFFSET 14

/*
* ----- GLOBAL CLASSES -----
*/
LPP lpp;


/*
* ----- GLOBAL VARIABLES -----
*/
Pose pose;
bool turn_hector_off = false;
float move_arm = 0;
float move_basket = 0;
int map_offset_x;
int map_offset_y;

/*
* ----- CALLBACK FUNCTION DEFINITIONS -----
*/
void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
void imuCB(const std_msgs::Float32MultiArray::ConstPtr& msg);
void arduinoCB(const std_msgs::Int16MultiArray::ConstPtr& msg);

/*
* ----- SERVICE FUNCTIONS -----
*/
bool getPoseSRV(robot::GetPoseSRV::Request &req,
         		 robot::GetPoseSRV::Response &res);
bool setCommandSRV(robot::CommandSRV::Request &req,
         		 	 robot::CommandSRV::Response &res);
void controllerCommandMotors(ros::Publisher& pub_motor_vel, bool stop_robot);

float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
} 


/*
* ----- MAIN -----
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "lpp_service");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	// subscribe to topics
	ros::Subscriber sub_pose = n.subscribe("slam_out_pose", 100, poseCB);
	ros::Subscriber sub_imu = n.subscribe("imu_euler", 100, imuCB);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 100, arduinoCB);
	
	// publisher of topics
	ros::Publisher pub_motor_vel = 
				n.advertise<std_msgs::Float32MultiArray>("motor_vel", 10);
				
	// provided services
	ros::ServiceServer srv_command = 
			n.advertiseService("controller_command_srv", setCommandSRV);
	ros::ServiceServer srv_pose = n.advertiseService("controller_get_pose_srv", 
																	getPoseSRV);
																	
	// create client
	ros::ServiceClient client_pause_hector = 
	 								n.serviceClient<robot::SetPoseSRV>("pause_hector");

	// retrieve parameters
	double map_start_x, map_start_y;
	if (n.getParam("/hector_mapping/map_start_x", map_start_x)) {
		ROS_INFO("controller: got param map_start_x");
	} else {
		ROS_ERROR("controller: failed to get param map_start_x");
		map_start_x = MAP_START_X;
	}
	if (n.getParam("/hector_mapping/map_start_y", map_start_y)) {
		ROS_INFO("controller: got param map_start_y");
	} else {
		ROS_ERROR("controller: failed to get param map_start_y");
		map_start_y = MAP_START_Y;
	}

	// set offset with respect to map border
	map_offset_x = int(MAP_SIZE*map_start_x);
	map_offset_y = int(MAP_SIZE*map_start_y);
				
	ros::Duration(2).sleep();

	int counter = 0;
	while(ros::ok()) {
		
		if(CONTROLLER_VERBOSE) {
			ROS_INFO_STREAM("\n----Controller: " << counter);
		}
		
		//controllerCommandMotors(pub_motor_vel, false);
  		
		ros::spinOnce();
      loop_rate.sleep();
		counter++;
	}

	// stopp robot
	controllerCommandMotors(pub_motor_vel, true);
}


/*
* ----- CALLBACK FUNCTIONS -----
*/
void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	float new_pose[3] = {0.0,0.0,0.0};

	// convert position in meters to pixels and subtract offset of LIDAR
	new_pose[0] = float(msg->pose.position.x*MAP_M2PIXEL + map_offset_x - ROBOT_CENTER_OFFSET);
	new_pose[1] = float(msg->pose.position.y*MAP_M2PIXEL + map_offset_y);

	// convert quaternions to radians
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
						  msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);	
	new_pose[2] = yaw;
	
	// update pose of LPP
	lpp.setPose(new_pose);

	if(CONTROLLER_VERBOSE_POSE_CB) {
		std::cout << "dm::poseCB::pose: (" << pose.position.x << "," << pose.position.y 
					 << "," << pose.heading << ")" << std::endl;
	}
}

void imuCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	// read message, gyro measurement is in degree/s
	float gyro_data[3] = {0,0,0};
	for(int i=0; i<3; i++) {
		gyro_data[i] = msg->data[i];
	}
	
	if(CONTROLLER_VERBOSE_IMU) {
		ROS_INFO_STREAM("controller::imuCB::meas = (" << gyro_data[0] << "," 
							 << gyro_data[1] << "," << gyro_data[2] << ")");
		ROS_INFO_STREAM("controller::imuCB: hector = " << turn_hector_off);
	}
	
	// update IMU measurement insdie LPP
	lpp.setIMUData(gyro_data);
	
	// turn hector off if rotational speed is too high
	if(abs(gyro_data[2])>5) {
		turn_hector_off = true;
	}
	if(abs(gyro_data[2])<2) {
		turn_hector_off = false;
	}
}

void arduinoCB(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	std::array<int,LPP_NB_SENSORS> meas;
	for(int i=0; i<LPP_NB_SENSORS; i++) {
		meas[i] = msg->data[i];
	}
	lpp.set_meas(meas);
}


/*
* ----- SERVICE FUNCTIONS -----
*/
bool getPoseSRV(robot::GetPoseSRV::Request &req,
         		 robot::GetPoseSRV::Response &res)
{
	//ROS_INFO_STREAM("Inside getPoseSRV");

	// get current pose
	std::array<float,3> pose = lpp.getPose();
	if(pose[0]<0 || pose[1]<0) {
		ROS_ERROR_STREAM("controller::getPoseSRV: position out of range");
		return false;
	}
	
	// return pose
	res.x = pose[0];
	res.y = pose[1];
	res.heading = pose[2];
	
	return true;
}

bool setCommandSRV(robot::CommandSRV::Request &req,
         		 	 robot::CommandSRV::Response &res)
{
	// set current state inside LPP
	lpp.set_dm_state(req.dm_state);
	
	// set arm and basket commands
	move_arm = float( req.move_arm );
	move_basket = float( req.move_basket );
	
	if(req.stop_motor) {
		lpp.stopMotors();
		
		if(CONTROLLER_VERBOSE_COMMAND) {
			ROS_INFO_STREAM("controller::setLPPCB: call lpp.stop_motor");
		}
	} else {
		// convert trajectory
		std::vector<cv::Point> trajectory;
		for(int i=0; i<req.nb_nodes; i++) {
			cv::Point point_temp;
			point_temp.x = req.trajectory_x[i];
			point_temp.y = req.trajectory_y[i];
			trajectory.push_back(point_temp);
		}

		// set trajectory in LPP
		lpp.setSetPoints(trajectory);
		
		if(CONTROLLER_VERBOSE_COMMAND) {
			ROS_INFO_STREAM("controller::setLPPCB: call lpp.setSetPoints");
		}
	}
	return true;
}

void controllerCommandMotors(ros::Publisher& pub_motor_vel, bool stop_robot)
{
	// get and update motor velocity
	std::array<float,4> motor_vel = lpp.getMotorVelocity();
	
	// define command to send
	std_msgs::Float32MultiArray msg_rasp2ard;
	if(stop_robot) {
		for(int i=0; i<4; i++) {
			msg_rasp2ard.data.push_back(0);
		}	
	
	} else {
		for(int i=0; i<4; i++) {
			msg_rasp2ard.data.push_back(motor_vel[i]);
		}
	}
	
	// define arm and basket commands
	msg_rasp2ard.data.push_back(move_arm);
	msg_rasp2ard.data.push_back(move_basket);
	
	// send motor commands to arduino
	pub_motor_vel.publish(msg_rasp2ard);
	
	// reset arm and basket commands (you do not want to sent them twice)
	move_arm = 0;
	move_basket = 0;

	if(CONTROLLER_VERBOSE_MOTORS) {
  		ROS_INFO_STREAM("LPP::motor_vel: (" << msg_rasp2ard.data[0] << "," 
  					 			<< msg_rasp2ard.data[1] << "," << msg_rasp2ard.data[2] 
  					 			<< "," << msg_rasp2ard.data[3] << ")");
  		ROS_INFO_STREAM("LPP: move_arm = " << msg_rasp2ard.data[4] << ", move_basket = " 
  					 			<< msg_rasp2ard.data[5]);
	}	
}

/*
* ----- TEST FUNCTIONS -----
*/
/*void testIMU(void)
{
	std::array<float,3> pose = {0,0,0};
	pose = lpp.getPose();
	
	if(CONTROLLER_VERBOSE_IMU) {
		ROS_INFO_STREAM("controller::testIMU::pose = (" << pose[0] << "," 
							 <<  pose[1] << "," << pose[2]*RAD2DEGREE << ")");
	}
}*/

/*void poseCovarianceCB(const 		
						geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	ROS_INFO_STREAM("Pose (x,y): (" 
						 << std::floor(msg->pose.pose.position.x) << ",\t" 
						 << std::floor(msg->pose.pose.position.y) << ")");
	ROS_INFO_STREAM("Pose variance (x,y,theta): (" 
						 << std::floor(msg->pose.covariance[0]) << ",\t" 
						 << std::floor(msg->pose.covariance[7]) << ",\t" 
						 << std::floor(msg->pose.covariance[35]) << ")");
}*/

/*void controllerProtectHector(ros::ServiceClient &client_pause_hector)
{
	// variable to remember last state
	static bool hector_last_state = turn_hector_off;
	
	// do nothing if turn_hector_off did not toggle
	if(hector_last_state == turn_hector_off) {
		return;
	}
	
	// define data for service
	robot::SetPoseSRV srv;
	if(turn_hector_off) {
		srv.request.turn_hector_off = true;
		
		if(CONTROLLER_VERBOSE_HECTOR) {
			ROS_INFO_STREAM("controller::controllerProtectHector: turn hector off");
		}
	} else {
		// get current pose estimation
		std::array<float,3> pose = lpp.getPose();
		
		srv.request.x = (pose[0] - MAP_OFFSET_X + ROBOT_CENTER_OFFSET)/MAP_M2PIXEL;
		srv.request.y = (pose[1] - MAP_OFFSET_X)/MAP_M2PIXEL;
		srv.request.heading = pose[2];		
		srv.request.turn_hector_off = false;
		
		if(CONTROLLER_VERBOSE_HECTOR) {
			ROS_INFO_STREAM("controller::controllerProtectHector: turn hector on");
		}
	}

	if(client_pause_hector.call(srv))
	{	
		// toggle hector_last_state
		hector_last_state = turn_hector_off;
	}
	else
	{
		ROS_ERROR("controller::controllerProtectHector: call client_pause_hector failed!");
	}
}*/


/*
* LOGGING FUNCTIONS
*/
/*
void initLogLPP(std::vector<cv::Point> trajectory)
{
	
	// create logging instance
	std::ofstream log;
	log.open("log.txt");

	// log set point distance threshold
	log << "--set point size\n";
	log << SET_POINT_DISTANCE_THRESHOLD << "\n";

	// log set points
	log << "--set points\n";
	for(int i=0; i<trajectory.size(); i++) {
	  log << trajectory[i].x << "," << trajectory[i].y << "\n";
	}

	// start logging poses
	log << "--poses\n";
	log.close();
}

void logLPP(void)
{
	// create logging instance
	std::ofstream log;
	log.open("log.txt", std::ios_base::app);

	std::array<float,4> vel = lpp.getMotorVelocity();
	std::array<float,3> pose = lpp.getPose();
	std::vector<cv::Point> set_points = lpp.getSetPoints();
	
	
	log << roundFloat(pose[0]) << "," << roundFloat(pose[1]) << "," 
										<< roundFloat(pose[2]) << "\n";
										
	if(CONTROLLER_VERBOSE_LOG) {
		ROS_INFO_STREAM("LPP::logLPP: log=(" << roundFloat(pose[0]) << "," << roundFloat(pose[1]) 
								<< "," << roundFloat(pose[2]) << ")");	
	}						

	log.close();
}

void initLPP(void)
{	
	// update pose of LPP
	float new_pose[3] = {60.0,60.0,0.0};
	lpp.setPose(new_pose);

	// set current state and trajectory inside LPP
	std::vector<cv::Point> trajectory;
	lpp.set_dm_state(DM_STATE_APPROACH);	
	cv::Point point_temp;
	point_temp.x = 0;
	point_temp.y = 0;
	trajectory.push_back(point_temp);
	point_temp.x = 80;
	point_temp.y = 80;
	trajectory.push_back(point_temp);
	
	lpp.setSetPoints(trajectory);
	
	// init. logging
	initLogLPP(trajectory);
}
*/
