/*
 Implementation of uav.h
 */

#include <math.h>
#include <stdlib.h>
#include "uav.h"
#include "coor.h"

static int counter = -1; // Counter for storing the tail
static int colorNumber = 0; // The tracker of colors already used for uavs
static int nextId = 0; // The id to assign to the next uav

float uavColors[MAX_COLORS][3] = {
    {1, 0, 0},{0, 1, 0},{0, 0, 1},{1, 1, 0},{1, 0, 1},{0, 1, 1},
    {1, 0.5, 0},{1, 0, 0.5},{0.5, 1, 0},{0, 1, 0.5},{0.5, 0, 1},{0, 0.5, 1},
    {1, 0.25, 0},{1, 0, 0.25},{0.25, 1, 0},{0, 1, 0.25},{0.25, 0, 1},{0, 0.25, 1},
    {1, 0.75, 0},{1, 0, 0.75},{0.75, 1, 0},{0, 1, 0.75},{0.75, 0, 1},{0, 0.75, 1},
    {.6, 0, 0},{0, .6, 0},{0, 0, .6},{.6, .6, 0},{.6, 0, .6},{0, .6, .6},
    {1, 1, 1},{.6, .6, .6}
};

UAV::UAV(double x, double y, double d) :
direction(d), active(true), curWaypoint(x,y), coor(x,y), idno(nextId), order(0)
{
    color[0] = uavColors[colorNumber][0];
    color[1] = uavColors[colorNumber][1];
    color[2] = uavColors[colorNumber][2];
    
    colorNumber++;
    nextId++;
    if (colorNumber >= MAX_COLORS)
        colorNumber = 0;
    
    //waypoints.push_back(Coordinate(rand()%400 - 200,rand()%400 - 200));
}

void UAV::step()
{
    
    // Turn and move the UAV
    direction+=order;
    if (direction >= 360)
        direction-=360;
    else if (direction < 0)
        direction+=360;
    
    coor.x += MAX_SPEED*cos(0.01745329252*direction);
    coor.y += MAX_SPEED*sin(0.01745329252*direction);
    
    // Save the tail
    if (counter%TAIL_RESOLUTION == 0)
    {
        oldCoor.push_back(Coordinate(coor.x,coor.y));
    }
    if (oldCoor.size() > TAIL_SIZE)
    {
        oldCoor.pop_front();
    }
}

void UAV::giveOrder(double o)
{
    order = o;
}

void UAV::nextWaypoint()
{
    if (waypoints.size()!=0)
    {
        curWaypoint.x = waypoints.front().x;
        curWaypoint.y = waypoints.front().y;
        waypoints.pop_front();
        
        waypoints.push_back(Coordinate(rand()%500 - 250,rand()%500 - 250));
    }
    else
    {
        active = false;
    }
}

bool UAV::getActive()
{
    return active;
}

void UAV::incrementCounter()
{
    counter++;
}
