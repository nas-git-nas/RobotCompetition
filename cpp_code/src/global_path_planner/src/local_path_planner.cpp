#include "std_msgs/String.h"
#include <vector>
#include <array>
#include <typeinfo>
#include <iostream>
#include <time.h>

#define WHEEL_RADIUS 0.05 // in m
#define INTER_WHEEL_DISTANCE 0.2 // in m
#define SET_POINT_DISTANCE_THRESHOLD 0.05 // in m
#define TURNING_UPPER_THRESHOLD 0.7854 // in rad, 45°
#define TURNING_LOWER_THRESHOLD 0.7854 // in rad, 45°
#define VEL_TURN_MAX 0.50 // in rad/s
#define VEL_TURN_MIN 0.25 // in rad/s
#define VEL_TURN_PID_KP 0.5
#define VEL_MOVE_MAX 0.40 // in rad/s
#define VEL_MOVE_MIN 0.20 // in rad/s
#define VEL_MOVE_BIAS 1.0 // in rad/s
#define VEL_MOVE_PID_KP 0.5
#define VEL_MOVE_PID_KI 0.2

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
        void calcMotorVelocity(void);
    
    private:
        float pose[3] = {};
        //std::vector<cv::Point> set_points;
        std::vector<PointTemp> set_points;
        std::array<float,4> motor_vel = {0,0,0,0}; // {right front, right back, left front, left back}
        time_t last_update_time;

        void updatePose(float delta_time);
        void robotStop(void);
        void robotMove(float theta_error, float theta_error_integration);
        void robotTurn(float theta_error);
        float limitVelocity(float vel, float max, float min);
};


//void LocalPathPlanner::setPoseAndSetPoints(std::vector<cv::Point> nodes, std::vector<int> shortest_path, float new_angle)
void LocalPathPlanner::setPoseAndSetPoints(std::vector<PointTemp> nodes, std::vector<int> shortest_path, float new_angle)
{
    pose[0] = nodes[shortest_path[0]].x;
    pose[1] = nodes[shortest_path[0]].y;
    pose[2] = new_angle;

    //skip first point of shortest_path because it's the current robot position
    set_points.clear();
    for(int i=1; i<shortest_path.size(); i++) { 
        set_points.push_back(nodes[shortest_path[i]]);
    }
}

std::array<float,4> LocalPathPlanner::getMotorVelocity(void)
{
    return motor_vel;
}

void LocalPathPlanner::calcMotorVelocity(void)
{
    static bool moving_state = false;
    static float theta_error_integration = 0;
    
    // measure time passed since last update and update pose
    clock_t current_time = clock();
    float delta_time = float(current_time-last_update_time)/CLOCKS_PER_SEC;
    updatePose(delta_time);
    last_update_time = delta_time;

    // calc. relative distance from current psition to first set point
    float error_x = set_points[0].x - pose[0];
    float error_y = set_points[0].y - pose[1];

    // verify distance to next set-point and update set-point list if necessary
    bool destination_reached = false;
    float distance = std::sqrt(error_x*error_x + error_y*error_y);
    while(distance < SET_POINT_DISTANCE_THRESHOLD) {
        if(set_points.size() <= 1) {
            destination_reached = true;
            break;
        }

        for(int i=0; i<set_points.size()-1; i++) {
            set_points[i] = set_points[i+1];
        }
    }

    if(destination_reached) {
        robotStop();
    } else {
        // calc. error in orientation (theta) and integrated theta
        float theta_error = atan2f(error_y, error_x) - pose[2];
        theta_error_integration += delta_time*theta_error;

        // verify if state should be changed: turning on spot or move to next set-point
        if(theta_error>TURNING_UPPER_THRESHOLD) {
            moving_state = false;
        } else if(theta_error<TURNING_LOWER_THRESHOLD) {
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

void LocalPathPlanner::updatePose(float delta_time)
{
    float delta_pos = delta_time*WHEEL_RADIUS*(motor_vel[0]+motor_vel[1]+motor_vel[2]+motor_vel[3])/4; // in m
    float delta_theta = delta_time*(motor_vel[0]+motor_vel[1]-motor_vel[2]-motor_vel[3])/(2*INTER_WHEEL_DISTANCE); // in rad
    pose[0] += cos(pose[2] + delta_theta/2)*delta_pos;
    pose[1] += sin(pose[2] + delta_theta/2)*delta_pos;
    pose[2] += delta_theta;
}

void LocalPathPlanner::robotStop(void)
{
    motor_vel[0] = 0;
    motor_vel[1] = 0;
    motor_vel[2] = 0;
    motor_vel[3] = 0;    
}

void LocalPathPlanner::robotMove(float theta_error, float theta_error_integration)
{
    float vel = VEL_MOVE_PID_KP*theta_error + VEL_MOVE_PID_KI*theta_error_integration;
    vel = limitVelocity(vel, VEL_MOVE_MAX, VEL_MOVE_MIN);

    motor_vel[0] = VEL_MOVE_BIAS + vel;
    motor_vel[1] = VEL_MOVE_BIAS + vel;
    motor_vel[2] = VEL_MOVE_BIAS - vel;
    motor_vel[3] = VEL_MOVE_BIAS - vel; 
}

void LocalPathPlanner::robotTurn(float theta_error)
{
    float vel = VEL_TURN_PID_KP*theta_error;
    vel = limitVelocity(vel, VEL_TURN_MAX, VEL_TURN_MIN);

    motor_vel[0] = vel;
    motor_vel[1] = vel;
    motor_vel[2] = -vel;
    motor_vel[3] = -vel;
}

float LocalPathPlanner::limitVelocity(float vel, float max, float min)
{
    if(vel>max) {
        vel = max;
    } else if(vel<min) {
        vel = min;
    }
    return vel;  
}

int main(int argc, char **argv)
{
    LocalPathPlanner local_path_planner;

    struct PointTemp p1;
    struct PointTemp p2;

    p1.x = 0;
    p1.y = 0;
    p2.x = 1;
    p2.y = 0;

    std::vector<PointTemp> nodes = {p1,p2};
    std::vector<int> shortest_path = {0,1};
    float new_angle = 0;

    local_path_planner.setPoseAndSetPoints(nodes, shortest_path, new_angle);
    local_path_planner.calcMotorVelocity();
    std::array<float,4> motor_vel = local_path_planner.getMotorVelocity();
    
    return 0;
}



