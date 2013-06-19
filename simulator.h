/*
 The simulator for simulating uav flight
 */

#ifndef __uavsim__simulator__
#define __uavsim__simulator__

#include <vector>
#include "uav.h"
#define WAYPOINT_SIZE 15
#define NEAR_MISS_SIZE 10
#define STEPS_PER_MSG 10

struct event {
    Coordinate coor, coor2;
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
    bool visualize; // Whether or not to add draw events
    
    Simulator();
    bool avoid(UAV *u); // Runs the avoidance algorithm on an uav
    bool step(); // Steps the simulator forward in time, returns false if ended
    void init(bool v); // Initializes the distances array and the visualize flag, must be called after all uavs have been added
    
    bool inverseprop(UAV *u);
};

#endif