#include <vector>
#include <array>
#include <typeinfo>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <cmath>
#include <cstdlib>

#define VERBOSE_LOCAL_PATH_PLANNER true
#define WHEEL_RADIUS 0.10 // in m
#define INTER_WHEEL_DISTANCE 0.2 // in m
#define SET_POINT_DISTANCE_THRESHOLD 0.05 // in m
#define TURNING_UPPER_THRESHOLD 0.7854 // in rad, 45°
#define TURNING_LOWER_THRESHOLD 0.7854 // in rad, 45°
#define VEL_TURN_MAX 0.2// in rad/s
#define VEL_TURN_MIN 0.1 // in rad/s
#define VEL_TURN_PID_KP 0.5
#define VEL_MOVE_MAX 2.5 // in rad/s

#define VEL_MOVE_BIAS 1.0 // in rad/s
#define VEL_MOVE_PID_KP 0.3
#define VEL_MOVE_PID_KI 0.05
#define PI 3.1415927

struct PointTemp
{
    float x;
    float y;
};


class LocalPathPlanner
{
    public:
        //void setPoseAndSetPoints(std::vector<cv::Point> nodes, std::vector<int> shortest_path, float new_angle);
        void setPoseAndSetPoints(std::vector<PointTemp> nodes, std::vector<int> shortest_path, float new_angle);
        std::array<float,4> getMotorVelocity(void);
        std::array<float,3> getPose(void);
        std::vector<PointTemp> getSetPoints(void);
        
    
    private:
        std::array<float,3> pose = {0,0,0};
        //std::vector<cv::Point> set_points;
        std::vector<PointTemp> set_points;
        std::array<float,4> motor_vel = {0,0,0,0}; // {right front, right back, left front, left back}
        time_t time_update_pose;
        bool destination_reached = true;

        void updateMotorVelocity(void);
        void updatePose(void);
        void robotStop(void);
        void robotMove(float theta_error, float theta_error_integration);
        void robotTurn(float theta_error);
        float limitVelocity(float vel, float max, float min);
        float limitAngle(float angle);
};

// helper functions
float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
}  


//void LocalPathPlanner::setPoseAndSetPoints(std::vector<cv::Point> nodes, std::vector<int> shortest_path, float new_angle)
void LocalPathPlanner::setPoseAndSetPoints(std::vector<PointTemp> nodes, std::vector<int> shortest_path, float new_angle)
{
    pose[0] = nodes[shortest_path[0]].x;
    pose[1] = nodes[shortest_path[0]].y;
    pose[2] = limitAngle(new_angle);

    //skip first point of shortest_path because it's the current robot position
    set_points.clear();
    for(int i=1; i<shortest_path.size(); i++) { 
        set_points.push_back(nodes[shortest_path[i]]);
    }
    destination_reached = false;

    // update time
    time_update_pose = clock();
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

std::vector<PointTemp> LocalPathPlanner::getSetPoints(void)
{
    return set_points;
}

void LocalPathPlanner::updateMotorVelocity(void)
{
    static bool moving_state = false;
    static float theta_error_integration = 0;
    static clock_t time_update_error = clock();
    
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
        time_update_error = clock();

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
        clock_t current_time = clock();
        float delta_time_error = float(current_time-time_update_error)/CLOCKS_PER_SEC;
        time_update_error = current_time;
        theta_error_integration += delta_time_error*theta_error;

        if(VERBOSE_LOCAL_PATH_PLANNER) {
            std::cout <<  "error = (" << error_x << "," << error_y << "," << theta_error << "), atan=" << atan2f(error_y, error_x) 
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
    clock_t current_time = clock();
    float delta_time = float(current_time-time_update_pose)/CLOCKS_PER_SEC;
    time_update_pose = current_time;

    // update pose
    float delta_pos = delta_time*WHEEL_RADIUS*(motor_vel[0]+motor_vel[1]+motor_vel[2]+motor_vel[3])/4; // in m
    float delta_theta = delta_time*(motor_vel[0]+motor_vel[1]-motor_vel[2]-motor_vel[3])/(2*INTER_WHEEL_DISTANCE); // in rad
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

int main(int argc, char **argv)
{
    LocalPathPlanner local_path_planner;

    struct PointTemp p1, p2, p3 ,p4;
    p1.x = 0;
    p1.y = 0;
    p2.x = 0.5;
    p2.y = 0;
    p3.x = 0.5;
    p3.y = 1;
    p4.x = -1;
    p4.y = 0;

    std::vector<PointTemp> nodes = {p1,p2,p3,p4};
    std::vector<int> shortest_path = {0,1,2,3};
    float new_angle = 0; //1.57;

    std::array<float,4> motor_vel;
    std::array<float,3> pose;
    std::vector<PointTemp>  set_points;

    std::ofstream log;
    log.open("log.txt");

    log << "--set point size\n";
    log << SET_POINT_DISTANCE_THRESHOLD << "\n";

    log << "--set points\n";
    for(int i=0; i<nodes.size(); i++) {
        log << roundFloat(nodes[shortest_path[i]].x) << "," << roundFloat(nodes[shortest_path[i]].y) << "\n";
    }
    std::cout << "node: (" << nodes[3].x << "," << nodes[3].y << ")" << std::endl;


    local_path_planner.setPoseAndSetPoints(nodes, shortest_path, new_angle);
    
    log << "--poses\n";
    for(int i=0; i<70; i++) {
        if(VERBOSE_LOCAL_PATH_PLANNER) {
            std::cout << "        i=" << i << std::endl;
        }

        motor_vel = local_path_planner.getMotorVelocity();
        pose = local_path_planner.getPose();
        set_points = local_path_planner.getSetPoints();

        if(VERBOSE_LOCAL_PATH_PLANNER) {
            std::cout << "pose: (" << pose[0] << "," << pose[1] << "," << rad2degrees(pose[2]) << ") vel: (" 
                        << motor_vel[0] << "," << motor_vel[1] << "," << motor_vel[2] << "," << motor_vel[3] << ") set-point: (" 
                        << set_points[0].x << "," << set_points[0].y << ")" << std::endl;
        }

        log << roundFloat(pose[0]) << "," << roundFloat(pose[1]) << "," << roundFloat(pose[2]) << "\n";
        std::cout << "i=" << i << std::endl;
        clock_t time_start = clock();
        float delta_time = 0;
        while(delta_time<0.5) {
            delta_time = float(clock()-time_start)/CLOCKS_PER_SEC;
        }
    }

    log.close();
    
    return 0;
}



