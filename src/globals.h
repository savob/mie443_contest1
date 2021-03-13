/* Global declaration file

All libraries are #include'd here so we only have to call 
#include "globals.h" once at the start of every file. It also
serves the more important purpose of allowing the global 
variables to be shared across multiple files without issue
(as long as it is used correctly! (as with most things)).

This allows callback functions to be defined seperately of 
the main file for easier collaboration.

To add global variables they need to be defined once in the
conventional sense in the .cpp file. Then it has to be once again
defined in this file, prefaced with "extern" so the compiler
knows that the global variable is defined in another place.

For example with the bumper pressed variable:

    // Conventional declaration in "bumper.cpp"
    bool any_bumper_pressed = false;

    // Extern declaration in this file
    extern bool any_bumper_pressed; 
    // Note the lack of a value assignment here

*/

#ifndef GLOBAL_HEADER
#define GLOBAL_HEADER

// Libraries we need
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>

// Some misc. stuff
const uint8_t N_BUMPER = 3;
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Motion variables
extern float angular;
extern float linear;
const float SLOW_SPIN = (M_PI/12);
const float FAST_SPIN = (M_PI/6);
const float SLOW_MOVE = 0.1;
const float FAST_MOVE = 0.25;
const float MAX_LIN = 0.25;
const float MAX_ROT = (M_PI/6);

// Robot state
extern u_int8_t state;
const u_int8_t SCAN_STATE = 0;
const u_int8_t EXPLORE_STATE = 1;
const u_int8_t BUMPER_STATE = 2;

// Odometery values
extern float posX, posY, yaw;

// Bumper values
extern uint8_t bumper[3];
extern bool anyBumperPressed;
extern uint8_t lastBumper;

// Laser related values
extern float minLaserDist;
extern int32_t nLasers, desiredNLasers; // Number of laser beams available, how many we will store given our desired range
extern int32_t desiredAngle;    // Desired view range aropund center axis, in degrees
extern float aveLaserDist;

// Motion related
const float WALL_CLEARANCE = 0.1;   // Distance to keep from obstacles
const float SLOW_DOWN_DIST = 0.3;     // Proximity to start slowing slow down at
const float EXPLORE_STEP = 0.75;     // Maximum exploration step to take
#endif 