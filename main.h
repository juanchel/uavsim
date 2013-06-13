#ifndef uavsim_main_h
#define uavsim_main_h

int uavDrawScale = 5; // The scale to visually draw the uavs
int waypointDrawScale = 5; // The scale to visually draw waypoints
int sleepRate = 20000; // Microseconds to sleep between each step

// The bounds of the drawing window
double xCeil = 250;
double xFloor = -250;
double yCeil = 250;
double yFloor = -250;

#endif
