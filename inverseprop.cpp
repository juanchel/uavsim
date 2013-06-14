/*
 Implementation of the inverse proportional navigation
 */
#include "simulator.h"
#define MAX_DIST_CHECK 100 // The maximum distance to require checking collision distance

bool Simulator::inverseprop(UAV *u)
{
    double minMiss = 1e99;
    
    // Identify the greatest threat
    for (UAV &v : uavs)
    {
        if (v.idno == u->idno)
            continue;
        
        if (v.distToUAV[u->idno] < MAX_DIST_CHECK)
        {
            u->step(10);
            return true;
        }
    }
    
    return false;
}