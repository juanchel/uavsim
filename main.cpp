#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif

#include <unistd.h>
#include <math.h>
#include <list>
#include <iterator>
#include <vector>
#include "uav.h"
#include "main.h"
#include "simulator.h"

int mouseX, mouseY;
int zoom = 0;

bool visualize = true;

Simulator sim;

double angleToRad(double angle) {return 0.01745329252*angle;}

/* Translates a simulated coordinate to one that fits properly in the window
 Axis should be 0 for x and 1 for y */
double coorToUnit (double coor, bool axis)
{
    double base;
    double dist;
    
    if (axis)
    {
        base = yFloor;
        dist = yCeil - yFloor;
    }
    else
    {
        base = xFloor;
        dist = xCeil - xFloor;
    }
    
    coor -= base;
    coor /= dist/2;
    coor -= 1;
    
    return coor;
}

// Reads and runs a course file
int readCourse (char* filename, Simulator* sim)
{
    sim->uavs.empty();
    
    std::vector<int> courseID;
    
    /* The vector courseID[x] stores the id defined by the course file of the xth plane in the simulation
     For many course files, courseID[x] == x */
    FILE *file = fopen(filename, "r");
    
    if (file != NULL)
    {
        char buffer[256];
        int id;
        double x;
        double y;
        double dir;
        
        int found = 0;
        
        bool latLng = true;
        double baseLat = -1;
        double baseLng = -1;
        
        while (fgets(buffer,sizeof(buffer), file))
        {
            if (buffer[0] >= 48 && buffer[0] < 58)
            {
                sscanf(buffer, "%d %lf %lf %lf", &id, &x, &y, &dir);
                
                /* If the course file is in latitude and longitude, convert it to meters
                 It is most accurate for course files that have latitiudes around 32-33 */
                if (latLng)
                {
                    if (baseLat == -1)
                    {
                        baseLat = y;
                        baseLng = x;
                    }
                    
                    y-=baseLat;
                    y*=93976.73;
                    
                    x-=baseLng;
                    x*=110895.58;
                }
                
                // Match the course id to the simulation id
                found = -1;
                for (int i=0; i<courseID.size(); i++)
                {
                    if (courseID[i] == id)
                        found = i;
                }
                
                // The the uav is not found in the simulation, create it, otherwise, add the waypoint
                if (found == -1)
                {
                    sim->uavs.push_back(UAV(x, y, dir));
                    courseID.push_back(id);
                }
                else
                {
                    sim->uavs[found].waypoints.push_back(Coordinate(x, y));
                }
            }
            else if (buffer[0] == '-')
            {
                if (buffer[1] == 'm')
                    latLng = false;
                else if (buffer[1] == 'l')
                    latLng = true;
            }
            else if (buffer[0] == '=' && buffer[1] == '=')
            {
                break;
            }
        }
        fclose(file);
        return 1;
    }
    
    return -1;
}

// Draws a grid
void drawGrid()
{
    double dist = xCeil-xFloor;
    double intensity = 0;
    
    glBegin(GL_LINES);
    if (dist < 800)
    {
        intensity = std::min(1.0, 1.0*(4-dist/200));
        glColor3f(intensity*0.25, intensity*0.1, intensity*0.25);
        for (int temp=ceil(xFloor/40)*40; temp<xCeil; temp+=40)
        {
            glVertex2f(coorToUnit(temp,0), coorToUnit(yFloor,1));
            glVertex2f(coorToUnit(temp,0), coorToUnit(yCeil,1));
        }
        for (int temp=ceil(yFloor/40)*40; temp<yCeil; temp+=40)
        {
            glVertex2f(coorToUnit(xFloor,0), coorToUnit(temp,1));
            glVertex2f(coorToUnit(xCeil,0), coorToUnit(temp,1));
        }
    }
    if (dist < 4000)
    {
        intensity = std::min(1.0, 1.0*(4-dist/1000));
        glColor3f(intensity*0.45, intensity*0.45, intensity*0.45);
        for (int temp=ceil(xFloor/200)*200; temp<xCeil; temp+=200)
        {
            glVertex2f(coorToUnit(temp,0), coorToUnit(yFloor,1));
            glVertex2f(coorToUnit(temp,0), coorToUnit(yCeil,1));
        }
        for (int temp=ceil(yFloor/200)*200; temp<yCeil; temp+=200)
        {
            glVertex2f(coorToUnit(xFloor,0), coorToUnit(temp,1));
            glVertex2f(coorToUnit(xCeil,0), coorToUnit(temp,1));
        }
    }
    if (dist < 40000)
    {
        intensity = std::min(1.0, 1.0*(4-dist/5000));
        glColor3f(intensity*0.3, intensity*0.9, intensity*0.3);
        for (int temp=ceil(xFloor/1000)*1000; temp<xCeil; temp+=1000)
        {
            glVertex2f(coorToUnit(temp,0), coorToUnit(yFloor,1));
            glVertex2f(coorToUnit(temp,0), coorToUnit(yCeil,1));
        }
        for (int temp=ceil(yFloor/1000)*1000; temp<yCeil; temp+=1000)
        {
            glVertex2f(coorToUnit(xFloor,0), coorToUnit(temp,1));
            glVertex2f(coorToUnit(xCeil,0), coorToUnit(temp,1));
        }
    }
    glEnd();
}

// Draws an uav
void drawUAV(UAV u)
{
    double d1 = angleToRad(u.direction);
    double d2 = angleToRad(u.direction+120);
    double d3 = angleToRad(u.direction-120);
    double scaleUAV = uavDrawScale/200.0;
    double scaleWP = waypointDrawScale*0.004;
    
    // The tail
    glBegin(GL_LINE_STRIP);
    double i=TAIL_SIZE-u.oldCoor.size();
    for (Coordinate c : u.oldCoor)
    {
        i++;
        glColor4f(u.color[0], u.color[1], u.color[2], i/TAIL_SIZE);
        glVertex2f(coorToUnit(c.x, 0), (coorToUnit(c.y, 1)));
    }
    glVertex2f(coorToUnit(u.coor.x, 0), coorToUnit(u.coor.y, 1));
    glEnd();

    // Waypoints
    glColor3f(u.color[0], u.color[1], u.color[2]);
    glBegin(GL_QUADS);
    glVertex2f(coorToUnit(u.curWaypoint.x,0)+scaleWP, coorToUnit(u.curWaypoint.y,1));
    glVertex2f(coorToUnit(u.curWaypoint.x,0), coorToUnit(u.curWaypoint.y,1)+scaleWP);
    glVertex2f(coorToUnit(u.curWaypoint.x,0)-scaleWP, coorToUnit(u.curWaypoint.y,1));
    glVertex2f(coorToUnit(u.curWaypoint.x,0), coorToUnit(u.curWaypoint.y,1)-scaleWP);
    glEnd();
    if (u.waypoints.size()!=0)
    {
        glBegin(GL_LINE_LOOP);
        glVertex2f(coorToUnit(u.waypoints.begin()->x,0)+scaleWP, coorToUnit(u.waypoints.begin()->y,1));
        glVertex2f(coorToUnit(u.waypoints.begin()->x,0), coorToUnit(u.waypoints.begin()->y,1)+scaleWP);
        glVertex2f(coorToUnit(u.waypoints.begin()->x,0)-scaleWP, coorToUnit(u.waypoints.begin()->y,1));
        glVertex2f(coorToUnit(u.waypoints.begin()->x,0), coorToUnit(u.waypoints.begin()->y,1)-scaleWP);
        glEnd();
    }
    
    // The UAV
    glColor3f(u.color[0], u.color[1], u.color[2]);
    glBegin(GL_TRIANGLES);
    glVertex2f(coorToUnit(u.coor.x, 0) + cos(d1)*scaleUAV, coorToUnit(u.coor.y, 1) + sin(d1)*scaleUAV);
    glVertex2f(coorToUnit(u.coor.x, 0) + cos(d2)*scaleUAV/2, coorToUnit(u.coor.y, 1) + sin(d2)*scaleUAV/2);
    glVertex2f(coorToUnit(u.coor.x, 0) + cos(d3)*scaleUAV/2, coorToUnit(u.coor.y, 1) + sin(d3)*scaleUAV/2);
    glEnd();
}

// Draws an event
void drawEvent(event e)
{
    switch (e.type)
    {
        case EVENT_REACHED_WAYPOINT:
        {
            double scaleWP = waypointDrawScale*0.004;
            double drawTime = (double)e.time/20.0;
            glColor4f(1, 1, 1, drawTime);
            glBegin(GL_QUADS);
            glVertex2f(coorToUnit(e.coor.x,0)+scaleWP*(5-drawTime*4), coorToUnit(e.coor.y,1));
            glVertex2f(coorToUnit(e.coor.x,0), coorToUnit(e.coor.y,1)+scaleWP*(5-drawTime*4));
            glVertex2f(coorToUnit(e.coor.x,0)-scaleWP*(5-drawTime*4), coorToUnit(e.coor.y,1));
            glVertex2f(coorToUnit(e.coor.x,0), coorToUnit(e.coor.y,1)-scaleWP*(5-drawTime*4));
            glEnd();
            break;
        }
        case EVENT_NEAR_MISS:
        {
            glBegin(GL_LINES);
            glColor3f(1,1,1);
            glVertex2f(coorToUnit(e.coor.x,0)+0.03, coorToUnit(e.coor.y,1)+0.03);
            glVertex2f(coorToUnit(e.coor.x,0)-0.03, coorToUnit(e.coor.y,1)-0.03);
            glVertex2f(coorToUnit(e.coor.x,0)-0.03, coorToUnit(e.coor.y,1)+0.03);
            glVertex2f(coorToUnit(e.coor.x,0)+0.03, coorToUnit(e.coor.y,1)-0.03);
            glEnd();
            break;
        }
    }
}

void display() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    sim.step();
    drawGrid();
    
    for (UAV u:sim.uavs)
    {
        if (u.active)
            drawUAV(u);
    }
    
    for (event e:sim.events)
    {
        drawEvent(e);
    }
    
    glFlush();
    usleep(sleepRate);
    
    if (zoom != 0)
    {
        double scale = zoom*8*(xCeil-xFloor)/glutGet(GLUT_WINDOW_WIDTH);
        xCeil -=scale;
        xFloor+=scale;
        yCeil -=scale;
        yFloor+=scale;
    }
    
    glutPostRedisplay();
}

void dragFunc(int x, int y)
{
    double xScale = (xCeil-xFloor)/glutGet(GLUT_WINDOW_WIDTH);
    double yScale = (xCeil-xFloor)/glutGet(GLUT_WINDOW_HEIGHT);
    
    xCeil += (mouseX-(double)x)*xScale;
    xFloor+= (mouseX-(double)x)*xScale;
    yCeil -= (mouseY-(double)y)*yScale;
    yFloor-= (mouseY-(double)y)*yScale;
    mouseX = x;
    mouseY = y;
}

void mouseFunc(int btn, int state, int x, int y)
{
    if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        mouseX = x;
        mouseY = y;
    }
}

void keyFunc(unsigned char key, int x, int y)
{
    if (key=='=') // Zoom in
    {
        zoom = 1;
    }
    else if (key=='-') // Zoom out
    {
        zoom = -1;
    }
}

void keyUpFunc(unsigned char key, int x, int y)
{
    if (key=='=' || key=='-') // Zoom in/out
    {
        zoom = 0;
    }
}

int main(int argc, char** argv) {
    
    readCourse((char*)"test.txt", &sim);
    
    sim.init(visualize);
    
    if (visualize)
    {
        glutInit(&argc, argv);
        glutInitWindowSize(800, 800);
        glutCreateWindow("UAV");
        glutDisplayFunc(display);
        glutMotionFunc(dragFunc);
        glutMouseFunc(mouseFunc);
        glutKeyboardFunc(keyFunc);
        glutKeyboardUpFunc(keyUpFunc);
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        glutMainLoop();
    }
    else
    {
        while (sim.step()) {}
    }

    return 0;
}