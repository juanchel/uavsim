/*
 The simulator for simulating uav flight
 */

#ifndef __uavsim__simulator__
#define __uavsim__simulator__

#include <vector>
#include "uav.h"
#define WAYPOINT_SIZE 15
#define NEAR_MISS_SIZE 15

struct event {
    Coordinate coor;
    int type;
    int time;
};

enum eventType {EVENT_REACHED_WAYPOINT, EVENT_NEAR_MISS};

double distance(Coordinate a, Coordinate b); // Calcuates the distance between two points
double dotProduct(Coordinate a, Coordinate b); // Calculates the dot product of two points
double angleTo(Coordinate a, Coordinate b); // Calculates the angle from a to b

class Simulator
{
public:
    int time; // The steps that have passed since the simulation begin
    std::vector<UAV> uavs; // Vector of uavs
    std::vector<event> events; // Vectors of events to draw
    
    Simulator();
    bool avoid(UAV *u); // Runs the avoidance algorithm on an uav
    bool step(); // Steps the simulator forward in time, returns false if ended
    void initDistances(); // Initializes the distances array, must be called after all uavs have been added
    
    bool inverseprop(UAV *u);
};

#endif