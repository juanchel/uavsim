/*
 A simulated uav
 */

#ifndef __uavsim__uav__
#define __uavsim__uav__

#include <list>
#include <vector>
#include "coor.h"
#define MAX_TURN 2.25
#define MAX_SPEED 1
#define MAX_COLORS 32
#define TAIL_RESOLUTION 12
#define TAIL_SIZE 24

class UAV
{
public:
    Coordinate coor; // The coordinates of the uav
    std::list<Coordinate> oldCoor; // List of old coordinates
    Coordinate curWaypoint; // The waypoint currently active
    std::list<Coordinate> waypoints; // List of waypoints to go
    std::vector<double> distToUAV; // List of distances to other uavs
    double direction; // The bearing, in degrees, of the uav
    bool active; // Whether the uav is active
    int idno; // The ID number of the uav
    
    UAV(double x, double y, double d);
    void step(double t); // Take a step of time, turning t degrees
    void nextWaypoint(); // Tells the uav to make the next waypoint active
    
    bool getActive(); // Returns whether or not the uav is active
    static void incrementCounter(); // Increments the counter by 1
    double color[3]; // The color of the uav
};

#endif