
#include "main.h"
#include "lpp.h"



void LPP::setPose(float *new_pose)
{
	// update pose
	pose[0] = new_pose[0];
	pose[1] = new_pose[1];
	pose[2] = limitAngle(new_pose[2]);

	// update time
	time_update_pose = ros::Time::now();
}

void LPP::setIMUData(float *gyro_data)
{
	for(int i=0; i<3; i++) {
		gyro[i] = gyro_data[i];
	}
}

void LPP::setSetPoints(std::vector<cv::Point> trajectory)
{
    //skip first point of shortest_path because it's the current 
    // robot position
    set_points.clear();
    for(int i=1; i<trajectory.size(); i++) { 
        set_points.push_back(trajectory[i]);
    }
    stop_robot = false;
}

void LPP::set_dm_state(uint8_t new_state)
{
	dm_state = new_state;
}

void LPP::set_meas(std::array<int,LPP_NB_SENSORS> new_meas)
{
 meas = new_meas;
}

void LPP::stopMotors(void)
{
	robotStop();
	stop_robot = true;
	//ROS_INFO_STREAM("lpp::stopMotors::stop_robot = " << stop_robot);
}

std::array<float,4> LPP::getMotorVelocity(void)
{
	// do nothing if robot is stopped
	if(stop_robot) {
		return motor_vel;
	}
	
	// enter different drive mode depending on decision maker state
	switch(dm_state) {	
		case DM_STATE_EXPLORE:
		case DM_STATE_MOVE:
		case DM_STATE_RETURN:
		case DM_STATE_RECYCLE:
			updateMotorVelocity();
			break;
		case DM_STATE_APPROACH:	
			updateApproachVelocity();
			break;
		default:
			robotStop();
			stop_robot = true;
	}
 
   /*ROS_INFO_STREAM("lpp::tegMotorVel::motor_vel: (" << motor_vel[0] << "," 
	  					 			<< motor_vel[1] << "," << motor_vel[2] 
	  					 			<< "," << motor_vel[3] << ")");
	ROS_INFO_STREAM("lpp::tegMotorVel::stop_robot: " << stop_robot);*/
   return motor_vel;
}

std::array<float,3> LPP::getPose(void)
{
    updatePose();
    return pose;
}

std::vector<cv::Point> LPP::getSetPoints(void)
{
    return set_points;
}

void LPP::updateMotorVelocity(void)
{
	//TODO: merge function with updateApproachVelocity

    static bool moving_state = false;
    
    // update pose and get time
    updatePose();

    // calc. relative distance from current psition to first set point
    float error_x = set_points[0].x - pose[0];
    float error_y = set_points[0].y - pose[1];

    // verify distance to next set-point and update set-point list if necessary
    float distance = sqrtf(error_x*error_x + error_y*error_y);
    while(distance < SET_POINT_DISTANCE_THRESHOLD) {

        // verify if destination is reached
        if(set_points.size() <= 1) {
            stop_robot = true;
            break;
        }

        // update set-point list
        for(int i=0; i<set_points.size()-1; i++) {
            set_points[i] = set_points[i+1];
        }
        set_points.pop_back();
        error_x = set_points[0].x - pose[0];
        error_y = set_points[0].y - pose[1];
        distance = sqrtf(error_x*error_x + error_y*error_y);
    }

    if(LPP_VERBOSE) {
        ROS_INFO_STREAM("pose: (" << pose[0] << "," << pose[1] << "," << pose[2] 
        				<< "), set_points[0]: (" << set_points[0].x 
                  << "," << set_points[0].y << "), nb. set_points: " 
                  << set_points.size() );
    } 

    if(stop_robot) {
        robotStop();
    } else {
        // calc. error in orientation (theta) and integrated theta
        float theta_error = limitAngle(atan2f(error_y, error_x) - pose[2]);

        if(LPP_VERBOSE) {
            std::cout <<  "error = (" << error_x << "," << error_y << "," << theta_error 
            			 << "), atan=" << atan2f(error_y, error_x) << std::endl;
        } 

        // verify if state should be changed: turning on spot or move to next set-point
        if(abs(theta_error)>TURNING_UPPER_THRESHOLD) {
            moving_state = false;
        } else if(abs(theta_error)<TURNING_LOWER_THRESHOLD) {
            moving_state = true;
        }

        // turn on spot or move to next set-point depending on state
        if(moving_state) {
            robotMove(theta_error, distance);
        } else {
            robotTurn(theta_error);
        }
    }
}

void LPP::updateApproachVelocity(void)
{
	if(stop_robot) {
	  robotStop();
	  return;
	} 

	static bool moving_state = false;

	// update pose
	updatePose();

	// calc. relative distance from current psition to first set point
	float error_x = set_points[0].x - pose[0];
	float error_y = set_points[0].y - pose[1];
	float distance = sqrtf(error_x*error_x + error_y*error_y);
	
	// calc. error in orientation (theta)
	float theta_error = limitAngle(atan2f(error_y, error_x) - pose[2]);

	if(LPP_VERBOSE) {
		std::cout <<  "error = (" << error_x << "," << error_y << "," << theta_error 
					 << "), atan=" << atan2f(error_y, error_x) << std::endl;
	}
	
	// verify if bottle is at optimal position
	if(abs(distance-LPP_ARM_LENGTH)<LPP_BOTTLE_DIST_THR && theta_error<LPP_BOTTLE_ANGLE_THR) {
	
		int left_meas = 0;
		int right_meas = 0;
		if(meas[0]>0 && meas[0]<LPP_MEAS_HEADING_THR) {
			left_meas += 1;
		}
		if(meas[1]>0 && meas[1]<LPP_MEAS_HEADING_THR) {
			left_meas += 1;
		}
		if(meas[2]>0 && meas[2]<LPP_MEAS_HEADING_THR) {
			left_meas += 1;
		}
		if(meas[4]>0 && meas[4]<LPP_MEAS_HEADING_THR) {
			right_meas += 1;
		}
		if(meas[5]>0 && meas[5]<LPP_MEAS_HEADING_THR) {
			right_meas += 1;
		}
		if(meas[6]>0 && meas[6]<LPP_MEAS_HEADING_THR) {
			right_meas += 1;
		}

		if(left_meas > right_meas) {
			robotCurve(10);
		} else if(left_meas < right_meas) {
			robotCurve(-10);
		} else {
			stop_robot = true;
			robotStop();		
		}		

		return;
	}	

	// verify if state should be changed: turning on spot or move to next set-point
	if(abs(theta_error)>APPROACH_UPPER_THRESHOLD) {
		moving_state = false;
	} else if(abs(theta_error)<APPROACH_LOWER_THRESHOLD) {
		moving_state = true;
	}

	// turn on spot or move to next set-point depending on state
	if(moving_state) {
		robotApproach(theta_error, distance);
	} else {
		robotTurn(theta_error);
	}
    
}

void LPP::updatePose()
{
	// measure time passed since last update and set current time
	ros::Time current_time = ros::Time::now();
	ros::Duration delta_time = current_time-time_update_pose;
	time_update_pose = current_time;

	// calc. change in position, in grid (cm)
	float delta_pos = delta_time.toSec() * WHEEL_RADIUS * M2GRID 
	* (motor_vel[0]+motor_vel[1]+motor_vel[2]+motor_vel[3])/4*VEL_MOVE_ADJUSTMENT;

	// calc. change in heading, in rad
	float delta_theta = 0;
	if(LPP_USE_IMU_TO_UPDATE_HEADING) {
		delta_theta = delta_time.toSec()*gyro[2]*DEGREE2RAD;
	} else {	
		delta_theta = delta_time.toSec()*(motor_vel[0]+motor_vel[1]-motor_vel[2]-motor_vel[3])
							*WHEEL_RADIUS/(2*INTER_WHEEL_DISTANCE*VEL_TURN_ADJUSTMENT);
	}
	
	// update pose						
	pose[0] += cos(pose[2] + delta_theta/2)*delta_pos;
	pose[1] += sin(pose[2] + delta_theta/2)*delta_pos;
	pose[2] += delta_theta;
	pose[2] = limitAngle(pose[2]);
}

void LPP::robotStop(void)
{
    motor_vel[0] = 0;
    motor_vel[1] = 0;
    motor_vel[2] = 0;
    motor_vel[3] = 0;

    if(LPP_VERBOSE) {
        std::cout << "robotStop" << std::endl;
    } 
}


void LPP::robotMove(float theta_error, float distance)
{
	float bias = theta_error*LPP_MOVE_BIAS_DELTA;
	bias = limitVelocity(bias, LPP_MOVE_BIAS_MAX, 0);
	
	float vel = distance*LPP_MOVE_DELTA + LPP_MOVE_ZERO;	
	float vel_r = vel*(1+bias);
	float vel_l = vel*(1-bias);
	vel_r = limitVelocity(vel_r, LPP_MOVE_VEL_MAX, LPP_MOVE_VEL_MIN);
	vel_l = limitVelocity(vel_l, LPP_MOVE_VEL_MAX, LPP_MOVE_VEL_MIN);

	motor_vel[0] = vel_r;
	motor_vel[1] = vel_r;
	motor_vel[2] = vel_l;
	motor_vel[3] = vel_l;

	if(LPP_VERBOSE) {
	  std::cout << "robotMove: vel=" << vel << std::endl;
	} 
}

void LPP::robotApproach(float theta_error, float distance)
{
	float bias = theta_error*LPP_APPR_BIAS_DELTA;
	bias = limitVelocity(bias, LPP_APPR_BIAS_MAX, 0);
	
	float vel = distance*LPP_APPR_DELTA + LPP_APPR_ZERO;

	// move straight if distance is larger than arm length, otherwise backwards
	float vel_r;
	float vel_l;	
	if(distance > LPP_ARM_LENGTH) {
		vel_r = vel*(1+bias);
		vel_l = vel*(1-bias);
	} else {
		vel_r = -vel*(1-bias);
		vel_l = -vel*(1+bias);
	}	

	vel_r = limitVelocity(vel_r, LPP_APPR_VEL_MAX, LPP_APPR_VEL_MIN);
	vel_l = limitVelocity(vel_l, LPP_APPR_VEL_MAX, LPP_APPR_VEL_MIN);

	motor_vel[0] = vel_r;
	motor_vel[1] = vel_r;
	motor_vel[2] = vel_l;
	motor_vel[3] = vel_l;

	if(LPP_VERBOSE) {
	  std::cout << "robotApproach: vel=" << vel << std::endl;
	} 
}

void LPP::robotTurn(float theta_error)
{
	float vel = 0;
	if(theta_error > 0) {
		vel = theta_error*LPP_TURN_DELTA + LPP_TURN_ZERO;
	} else {
		vel = theta_error*LPP_TURN_DELTA - LPP_TURN_ZERO;
	}	
	float vel_r = vel;
	float vel_l = -vel;
	vel_r = limitVelocity(vel_r, LPP_TURN_VEL_MAX, LPP_TURN_VEL_MIN);
	vel_l = limitVelocity(vel_l, LPP_TURN_VEL_MAX, LPP_TURN_VEL_MIN);

	motor_vel[0] = vel_r;
	motor_vel[1] = vel_r;
	motor_vel[2] = vel_l;
	motor_vel[3] = vel_l;

    if(LPP_VERBOSE) {
        std::cout << "robotTurn" << std::endl;
    } 
}

void LPP::robotCurve(float theta_error)
{
	float vel_left = 0;
	float vel_right = 0;
	if(theta_error>0) {
		vel_right = -LPP_CURVE_VEL_MIN;
		vel_left = -LPP_CURVE_VEL_MAX;
	} else {
		vel_right = -LPP_CURVE_VEL_MAX;
		vel_left = -LPP_CURVE_VEL_MIN;
	}

	motor_vel[0] = vel_right;
	motor_vel[1] = vel_right;
	motor_vel[2] = vel_left;
	motor_vel[3] = vel_left;
	
}

float LPP::limitVelocity(float vel, float max, float min)
{
    if(vel>max) {
        vel = max;
    } else if(vel<-max) {
        vel = -max;
    } else if(vel<min && vel>0) {
        vel = min;
    } else if(vel>-min && vel<0) {
        vel = -min;
    }
    return vel;  
}

float LPP::limitAngle(float angle)
{
    while(angle>PI) {
        angle -= 2*PI;
    }
    while(angle<-PI) {
        angle += 2*PI;
    }
    return angle;
}

/*
void LPP::robotMove(float theta_error, float distance)
{
    float vel = VEL_MOVE_PID_KP*theta_error;
    
    vel = limitVelocity(vel, VEL_MOVE_MAX, 0);

    motor_vel[0] = VEL_MOVE_BIAS + vel;
    motor_vel[1] = VEL_MOVE_BIAS + vel;
    motor_vel[2] = VEL_MOVE_BIAS - vel;
    motor_vel[3] = VEL_MOVE_BIAS - vel;

    if(LPP_VERBOSE) {
        std::cout << "robotMove: vel=" << vel << std::endl;
    } 
}

void LPP::robotMoveBack(float theta_error, float distance)
{
    float vel = VEL_MOVE_PID_KP*theta_error;
    vel = limitVelocity(vel, VEL_MOVE_MAX, 0);

    motor_vel[0] = -VEL_MOVE_BIAS + vel;
    motor_vel[1] = -VEL_MOVE_BIAS + vel;
    motor_vel[2] = -VEL_MOVE_BIAS - vel;
    motor_vel[3] = -VEL_MOVE_BIAS - vel;

    if(LPP_VERBOSE) {
        std::cout << "robotMove: vel=" << vel << std::endl;
    } 
}

void LPP::robotTurn(float theta_error)
{
    float vel = VEL_TURN_PID_KP*theta_error;
    vel = limitVelocity(vel, VEL_TURN_MAX, VEL_TURN_MIN);

    motor_vel[0] = vel;
    motor_vel[1] = vel;
    motor_vel[2] = -vel;
    motor_vel[3] = -vel;

    if(LPP_VERBOSE) {
        std::cout << "robotTurn" << std::endl;
    } 
}

void LPP::robotCurve(float theta_error)
{
	float vel_left = 0;
	float vel_right = 0;
	if(theta_error>0) {
		vel_right = VEL_TURN_MIN;
		vel_left = -VEL_TURN_MAX;
	} else {
		vel_right = -VEL_TURN_MAX;
		vel_left = VEL_TURN_MIN;
	}

	motor_vel[0] = vel_right;
	motor_vel[1] = vel_right;
	motor_vel[2] = vel_left;
	motor_vel[3] = vel_left;
	
}

*/
