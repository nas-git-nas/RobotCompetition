#include "ros/ros.h"
#include "robot/GetPoseSRV.h"
#include "robot/SetPoseSRV.h"
#include "robot/SetTrajectorySRV.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBool.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>

#include "main.h"
#include "lpp.h"


#define MAP_SIZE_M 5
#define MAP_RESOLUTION 0.01
#define MAP_START_X 0.5
#define MAP_START_Y 0.5
#define MAP_OFFSET_X int(MAP_SIZE_M*MAP_START_X/MAP_RESOLUTION)
#define MAP_OFFSET_Y int(MAP_SIZE_M*MAP_START_Y/MAP_RESOLUTION)
#define MAP_M2PIXEL float(1/MAP_RESOLUTION)
//TODO: verify
#define ROBOT_CENTER_OFFSET 15

/*
* ----- GLOBAL CLASSES -----
*/
LPP lpp;


/*
* ----- GLOBAL VARIABLES -----
*/
Pose pose;
bool turn_hector_off = false;

/*
* ----- CALLBACK FUNCTIONS -----
*/

void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	float new_pose[3] = {0.0,0.0,0.0};

	// convert position in meters to pixels and subtract offset of LIDAR
	new_pose[0] = float(msg->pose.position.x*MAP_M2PIXEL + MAP_OFFSET_X - ROBOT_CENTER_OFFSET);
	new_pose[1] = float(msg->pose.position.y*MAP_M2PIXEL + MAP_OFFSET_Y);

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

void poseCovarianceCB(const 		
						geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	/*ROS_INFO_STREAM("Pose (x,y): (" 
						 << std::floor(msg->pose.pose.position.x) << ",\t" 
						 << std::floor(msg->pose.pose.position.y) << ")");
	ROS_INFO_STREAM("Pose variance (x,y,theta): (" 
						 << std::floor(msg->pose.covariance[0]) << ",\t" 
						 << std::floor(msg->pose.covariance[7]) << ",\t" 
						 << std::floor(msg->pose.covariance[35]) << ")");*/
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

bool setTrajectorySRV(robot::SetTrajectorySRV::Request &req,
         		 		 robot::SetTrajectorySRV::Response &res)
{
	if(req.stop_motor) {
		lpp.stopMotors();
		
		if(CONTROLLER_VERBOSE) {
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
		
		if(CONTROLLER_VERBOSE) {
			ROS_INFO_STREAM("controller::setLPPCB: call lpp.setSetPoints");
		}
	}
	return true;
}


/*
* ----- CONTROLLER FUNCTIONS -----
*/

void controllerCommandMotors(ros::Publisher& pub_motor_vel)
{
	// get and update motor velocity
	std::array<float,4> motor_vel = lpp.getMotorVelocity();

	// send motor commands to arduino
	std_msgs::Float32MultiArray msg_rasp2ard;
	for(int i=0; i<4; i++) {
		msg_rasp2ard.data.push_back(motor_vel[i]);
	}
	pub_motor_vel.publish(msg_rasp2ard);

	if(CONTROLLER_VERBOSE) {
  		ROS_INFO_STREAM("LPP::motor_vel: (" << msg_rasp2ard.data[0] << "," 
  					 			<< msg_rasp2ard.data[1] << "," << msg_rasp2ard.data[2] 
  					 			<< "," << msg_rasp2ard.data[3] << ")" << std::endl);
	}	
}

void controllerProtectHector(ros::ServiceClient &client_pause_hector)
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
}



/*
* ----- TEST FUNCTIONS -----
*/
void testIMU(void)
{
	std::array<float,3> pose = {0,0,0};
	pose = lpp.getPose();
	
	if(CONTROLLER_VERBOSE_IMU) {
		ROS_INFO_STREAM("controller::testIMU::pose = (" << pose[0] << "," 
							 <<  pose[1] << "," << pose[2]*RAD2DEGREE << ")");
	}
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
	ros::Subscriber sub_pose_cov = n.subscribe("poseupdate", 100, poseCovarianceCB);
	ros::Subscriber sub_imu = n.subscribe("imu_euler", 100, imuCB);
	
	// publisher of topics
	ros::Publisher pub_motor_vel = 
				n.advertise<std_msgs::Float32MultiArray>("motor_vel", 10);
				
	// provided services
	ros::ServiceServer srv_trajectory = 
			n.advertiseService("controller_set_trajectory_srv", setTrajectorySRV);
	ros::ServiceServer srv_pose = n.advertiseService("controller_get_pose_srv", 
																	getPoseSRV);
																	
	// create client
	ros::ServiceClient client_pause_hector = 
	 								n.serviceClient<robot::SetPoseSRV>("pause_hector");

				
	ros::Duration(2).sleep();

	int counter = 0;
	while(ros::ok()) {
		
		if(CONTROLLER_VERBOSE) {
			ROS_INFO_STREAM("\n----Controller: " << counter);
		}
		
		//controllerCommandMotors(pub_motor_vel);
		//controllerProtectHector(client_pause_hector);
		//testIMU();
  		
		ros::spinOnce();
      loop_rate.sleep();
		counter++;
	}


	return 0;
}


/*float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
} 

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

void logLPP(LPP &lpp)
{
	// create logging instance
	std::ofstream log;
	log.open("log.txt", std::ios_base::app);


	std::array<float,3> pose = lpp.getPose();
	std::vector<cv::Point> set_points = lpp.getSetPoints();
	
	
	log << roundFloat(pose[0]) << "," << roundFloat(pose[1]) << "," 
										<< roundFloat(pose[2]) << "\n";
										

	log.close();
}*/
