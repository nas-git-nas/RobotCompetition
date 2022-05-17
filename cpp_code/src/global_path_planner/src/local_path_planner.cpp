
#include "local_path_planner.h"


 


void LocalPathPlanner::setPoseAndSetPoints(
				std::vector<cv::Point> trajectory, float new_heading)
{
    pose[0] = trajectory[0].x;
    pose[1] = trajectory[0].y;
    pose[2] = limitAngle(new_heading);

    //skip first point of shortest_path because it's the current 
    // robot position
    set_points.clear();
    for(int i=1; i<trajectory.size(); i++) { 
        set_points.push_back(trajectory[i]);
    }
    destination_reached = false;

    // update time
    time_update_pose = ros::Time::now();
}

std::array<float,4> LocalPathPlanner::getMotorVelocity(void)
{
    updateMotorVelocity();
    return motor_vel;
}

std::array<float,3> LocalPathPlanner::getPose(void)
{
    updatePose();
    return pose;
}

std::vector<cv::Point> LocalPathPlanner::getSetPoints(void)
{
    return set_points;
}

void LocalPathPlanner::updateMotorVelocity(void)
{
    static bool moving_state = false;
    static float theta_error_integration = 0;
    static ros::Time time_update_error = ros::Time::now();
    
    // update pose and get time
    updatePose();

    // calc. relative distance from current psition to first set point
    float error_x = set_points[0].x - pose[0];
    float error_y = set_points[0].y - pose[1];

    // verify distance to next set-point and update set-point list if necessary
    float distance = sqrtf(error_x*error_x + error_y*error_y);
    while(distance < SET_POINT_DISTANCE_THRESHOLD) {
        // reset error integration
        theta_error_integration = 0; 
        time_update_error = ros::Time::now();

        // verify if destination is reached
        if(set_points.size() <= 1) {
            destination_reached = true;
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

    if(VERBOSE_LOCAL_PATH_PLANNER) {
        std::cout << "pose: (" << pose[0] << "," << pose[1] << "," << pose[2] << "), set_points[0]: " << set_points[0].x 
                  << "," << set_points[0].x << ")" << std::endl;
    } 

    if(destination_reached) {
        robotStop();
    } else {
        // calc. error in orientation (theta) and integrated theta
        float theta_error = limitAngle(atan2f(error_y, error_x) - pose[2]);
        ros::Time current_time = ros::Time::now();
        ros::Duration delta_time_error = current_time-time_update_error;
        time_update_error = current_time;
        theta_error_integration != delta_time_error.toSec()*theta_error;

        if(VERBOSE_LOCAL_PATH_PLANNER) {
            std::cout <<  "error = (" << error_x << "," << error_y << "," << theta_error 
            			 << "), atan=" << atan2f(error_y, error_x) 
                      <<  ", theta_error_integration: " << theta_error_integration << std::endl;
        } 

        // verify if state should be changed: turning on spot or move to next set-point
        if(abs(theta_error)>TURNING_UPPER_THRESHOLD) {
            moving_state = false;
        } else if(abs(theta_error)<TURNING_LOWER_THRESHOLD) {
            moving_state = true;
        }

        // turn on spot or move to next set-point depending on state
        if(moving_state) {
            robotMove(theta_error, theta_error_integration);
        } else {
            robotTurn(theta_error);
        }
    }
}

void LocalPathPlanner::updatePose()
{
    // measure time passed since last update and set current time
    ros::Time current_time = ros::Time::now();
    ros::Duration delta_time = current_time-time_update_pose;
    time_update_pose = current_time;

    // update pose
    float delta_pos = delta_time.toSec() * WHEEL_RADIUS * M2GRID 
    	* (motor_vel[0]+motor_vel[1]+motor_vel[2]+motor_vel[3])/4; // in grid
    float delta_theta = delta_time.toSec()*(motor_vel[0]+motor_vel[1]-motor_vel[2]-motor_vel[3])
    							*WHEEL_RADIUS/(2*INTER_WHEEL_DISTANCE*VEL_TURN_ADJUSTMENT); // in rad
    pose[0] += cos(pose[2] + delta_theta/2)*delta_pos;
    pose[1] += sin(pose[2] + delta_theta/2)*delta_pos;
    pose[2] += delta_theta;
    pose[2] = limitAngle(pose[2]);
}

void LocalPathPlanner::robotStop(void)
{
    motor_vel[0] = 0;
    motor_vel[1] = 0;
    motor_vel[2] = 0;
    motor_vel[3] = 0;

    if(VERBOSE_LOCAL_PATH_PLANNER) {
        std::cout << "robotStop" << std::endl;
    } 
}

void LocalPathPlanner::robotMove(float theta_error, float theta_error_integration)
{
    float vel = VEL_MOVE_PID_KP*theta_error + VEL_MOVE_PID_KI*theta_error_integration;
    vel = limitVelocity(vel, VEL_MOVE_MAX, 0);

    motor_vel[0] = VEL_MOVE_BIAS + vel;
    motor_vel[1] = VEL_MOVE_BIAS + vel;
    motor_vel[2] = VEL_MOVE_BIAS - vel;
    motor_vel[3] = VEL_MOVE_BIAS - vel;

    if(VERBOSE_LOCAL_PATH_PLANNER) {
        std::cout << "robotMove: vel=" << vel << std::endl;
    } 
}

void LocalPathPlanner::robotTurn(float theta_error)
{
    float vel = VEL_TURN_PID_KP*theta_error;
    vel = limitVelocity(vel, VEL_TURN_MAX, VEL_TURN_MIN);

    motor_vel[0] = vel;
    motor_vel[1] = vel;
    motor_vel[2] = -vel;
    motor_vel[3] = -vel;

    if(VERBOSE_LOCAL_PATH_PLANNER) {
        std::cout << "robotTurn" << std::endl;
    } 
}

float LocalPathPlanner::limitVelocity(float vel, float max, float min)
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

float LocalPathPlanner::limitAngle(float angle)
{
    while(angle>PI) {
        angle -= 2*PI;
    }
    while(angle<-PI) {
        angle += 2*PI;
    }
    return angle;
}




