/*
 Implementation of the inverse proportional navigation
 */
#include <math.h>
#include "simulator.h"
#define MAX_DIST_CHECK 100 // The maximum distance to require checking for collision
#define MAX_MISS_AVOID 1200 // The maximum miss distance squared to require avoidance
#define MAX_TIME_AVOID 50 // The maximum miss time to go to require avoidance

using namespace std;

bool Simulator::inverseprop(UAV *u)
{
    double minMiss = MAX_MISS_AVOID;
    UAV* threat = NULL;
    
    // Identify the greatest threat
    for (UAV &v : uavs)
    {
        // If the uav to check is the checking uav, skip it
        if (v.idno == u->idno)
            continue;
        
        Coordinate uDir(cos(0.01745329252*u->direction), sin(0.01745329252*u->direction));
        Coordinate vDir(cos(0.01745329252*v.direction), sin(0.01745329252*v.direction));
        
        double angleDiff = fabs(v.direction - u->direction);
        if (angleDiff > 180)
            angleDiff = 360-angleDiff;
        
        // If the other UAV is not within a certain check threshold, ignore it
        if (v.distToUAV[u->idno] < MAX_DIST_CHECK && angleDiff > 10 && angleDiff < 170)
        {
            double toGo = -1.0 * dotProduct(u->coor-v.coor, uDir-vDir) / dotProduct(uDir-vDir, uDir-vDir) / (double)MAX_SPEED;
            double missDist = dotProduct(uDir-vDir, uDir-vDir) * pow(MAX_SPEED * toGo, 2) + 2 * dotProduct(u->coor-v.coor, uDir-vDir) * MAX_SPEED * toGo +dotProduct(u->coor-v.coor, u->coor-v.coor);
            
            if (missDist < minMiss && toGo > 0 && toGo < MAX_TIME_AVOID)
            {
                minMiss = missDist;
                threat = &v;
            }
        }
    }
    
    // If a potential threat was found, avoid it
    if (threat != NULL)
    {
        // Decide which way to turn
        double angleBetween = angleTo(u->coor, threat->coor);
        double thetaU = fabs(u->direction - angleBetween);
        thetaU = (thetaU > 180) ? 360 - thetaU : thetaU;
        
        angleBetween -= 180;
        if (angleBetween < 0)
            angleBetween += 360;
        double thetaV = fabs(threat->direction - angleBetween);
        thetaV = (thetaV > 180) ? 360 - thetaV : thetaV;
        
        double comp = sin(0.01745329252*thetaU) - sin(0.01745329252*thetaV);
        
        double turn = (comp < 0) ? MAX_TURN : -1*MAX_TURN;
        if (turn < 0.01)
            turn = MAX_TURN;
    
        u->giveOrder(turn);
        return true;
    }
    return false;
}