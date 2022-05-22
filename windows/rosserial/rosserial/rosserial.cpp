
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <windows.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "global_path_planner/WindowsMsg.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> 

#define SET_POINT_DISTANCE_THRESHOLD 5

using std::string;

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
}

void logPosition(std::vector<cv::Point> nodes, std::vector<int> path)
{
    static bool log_position_initialized = false;

    // create logging instance
    std::ofstream log;
    log.open("log_position.txt", std::ios_base::app);

    if (!log_position_initialized) {
        // log set point distance threshold
        log << "--set point size\n";
        log << SET_POINT_DISTANCE_THRESHOLD << "\n";

        // log set points
        log << "--set points\n";
        for (int i = 0; i < path.size(); i++) {
            log << nodes[path[i]].x << "," << nodes[path[i]].y << "\n";
        }

        // start logging poses
        log << "--poses\n";

        log_position_initialized = true;
    }

    log << roundFloat(nodes[0].x) << "," << roundFloat(nodes[0].y) << ","
        << roundFloat(0) << "\n";

    log.close();
}


int drawColor(int iter, int max)
{
    int color = int(iter * 200.0 / (max-1) + 55.0);
    //std::cout << "(iter/max): (" << iter << "/" << max<< "), color: " << int(color) << std::endl;
    return color;
}


void drawMap(std::vector<cv::Point> nodes, std::vector<int> polygons, std::vector<int> path)
{
    cv::Mat map(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // draw polygons
   int polygon_index = 0;
    for (int i = 0; i < nodes.size() - 1; i++) {
        if (polygons[i] == polygons[i + 1]) {
            cv::line(map, nodes[i], nodes[i + 1], cv::Scalar(255, 0, 0), 1, cv::LINE_4);
        }
        else {
            cv::line(map, nodes[i], nodes[polygon_index], cv::Scalar(255, 0, 0), 1, cv::LINE_4);
            polygon_index = i + 1;
        }
    }

    // draw shortest path
    for (int i = 0; i < path.size(); i++) {
        cv::Scalar color(0, 0, drawColor(i, path.size()));

        if (i < path.size()-1) {
            cv::line(map, nodes[path[i]], nodes[path[i + 1]], color, 1, cv::LINE_4);
        }
        cv::circle(map, nodes[path[i]], 5, color, cv::FILLED);
    }

    //cv::putText(map, "current pos", nodes[0], cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1);

    cv::imshow("map_gui", map);
    cv::waitKey(1);
}

void raspberryCB(const global_path_planner::WindowsMsg& msg)
{

    std::vector<cv::Point> nodes;
    std::vector<int> polygons;
    std::vector<int> path;

    for (int i = 0; i < msg.nb_nodes; i++) {
        cv::Point temp;
        temp.x = msg.nodes_x[i];
        temp.y = msg.nodes_y[i];
        nodes.push_back(temp);
        polygons.push_back(msg.polygons[i]);
    }

    for (int i = 0; i < msg.nb_path_nodes; i++) {
        path.push_back(msg.path[i]);
        std::cout << "Point(" << i << ") = (" << nodes[path[i]].x << "," << nodes[path[i]].y << ")" << std::endl;
    }

    drawMap(nodes, polygons, path);
    logPosition(nodes, path);
}


int main()
{
    ros::NodeHandle nh;
    char ros_master[9] = "10.0.0.9";

    printf("Connecting to server at %s\n", ros_master);
    nh.initNode(ros_master);

    ros::Subscriber <global_path_planner::WindowsMsg>poseSub("windows_pub", &raspberryCB);
    nh.subscribe(poseSub);

    // init. opencv window
    cv::namedWindow("map_gui", cv::WINDOW_AUTOSIZE);
    cv::Mat map(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // init. log position file
    std::ofstream log;
    log.open("log_position.txt");
    log.close();

    while (1)
    {
        nh.spinOnce();
        Sleep(1000);

        if ((char)cv::waitKey(1) == 101) { // exit it "e" is pressed
            break;
        }

    }

    // Destroy the windows 
    cv::destroyWindow("map_gui");

    printf("All done!\n");
    return 0;
}