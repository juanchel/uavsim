/*
 Implemetation of simulator.h
 */

#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include "simulator.h"

double distance(Coordinate a, Coordinate b)
{
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

Simulator::Simulator() {}

bool Simulator::step()
{
    UAV::incrementCounter();
    time++;
    
    for (UAV &u : uavs)
    {
        if (u.active)
        {
            if (!avoid(&u))
            {
                // Calculate the angle between the uav and waypoint
                double deltaX = u.curWaypoint.x - u.coor.x;
                double deltaY = u.curWaypoint.y - u.coor.y;
                double toWaypoint = atan2(deltaY, deltaX)*57.295779513;
                
                if (toWaypoint < 0)
                    toWaypoint+=360;
                
                // Calculate the amount to turn
                double turn = fabs((double)toWaypoint - u.direction);
                
                if (turn > 180)
                    turn = 360-turn;
                
                if (turn > MAX_TURN)
                    turn = MAX_TURN;
                
                // If the waypoint cannot be turned into, turn the other way
                // 57.2957795131 rounded up to 60
                double radius = 60/MAX_TURN*MAX_SPEED;
                Coordinate centerA(sin(0.01745329252*u.direction)*radius, -1*cos(0.01745329252*u.direction)*radius);
                Coordinate centerB(-1*centerA.x, -1*centerA.y);
                
                centerA.x+=u.coor.x;
                centerB.x+=u.coor.x;
                centerA.y+=u.coor.y;
                centerB.y+=u.coor.y;
                
                if (distance(centerA, u.curWaypoint)<radius || distance(centerB, u.curWaypoint)<radius)
                {
                    turn *= -1;
                }
                
                // Calculate the direction to turn
                if (u.direction < 180)
                {
                    if (toWaypoint > u.direction && toWaypoint < u.direction+180)
                        u.step(turn);
                    else
                        u.step(-1*turn);
                }
                else
                {
                    if (toWaypoint < u.direction && toWaypoint > u.direction-180)
                        u.step(-1*turn);
                    else
                        u.step(turn);
                }
            }
            
            if (distance(u.coor, u.curWaypoint) < WAYPOINT_SIZE)
            {
                if (time > 2)
                {
                    event temp;
                    temp.coor = Coordinate(u.curWaypoint.x, u.curWaypoint.y);
                    temp.time = 20;
                    temp.type = EVENT_REACHED_WAYPOINT;
                    events.push_back(temp);
                }
                u.nextWaypoint();
            }
        }
    }
    
    // Calculate near misses
    for (UAV &u : uavs)
    {
        for (UAV &v : uavs)
        {
            if (&u==&v)
                break;
            
            u.distToUAV[v.idno] = distance(u.coor, v.coor);
            v.distToUAV[u.idno] = u.distToUAV[v.idno];
            
            if (u.distToUAV[v.idno] < NEAR_MISS_SIZE)
            {
                event temp;
                temp.coor = Coordinate((u.coor.x + v.coor.x)/2, (u.coor.y + v.coor.y)/2);
                temp.time = 2;
                temp.type = EVENT_NEAR_MISS;
                events.push_back(temp);
            }
        }
    }
    
    // Manage the events to draw
    for (int i = (int)events.size()-1; i>=0; i--)
    {
        events[i].time--;
        if (events[i].time <= 0)
        {
            events.erase(events.begin()+i);
        }
    }
    
    return true;
}

bool Simulator::avoid(UAV *u)
{
    return inverseprop(u);
}

void Simulator::initDistances()
{
    for (UAV &u : uavs)
    {
        u.distToUAV.resize(uavs.size());
    }
}