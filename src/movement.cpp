#include "movement.h"

// Movement values
float angular = 0.0; // Global
float linear = 0.0;  // Global

// File scoped globals
float distanceRemaining, prevX, prevY;
float rotationRemaining, prevYaw;
float rotMaintain, linMaintain;

// Odometery values
float posX = 0.0, posY = 0.0, yaw = 0.0;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

bool monitorMotion ()
{
    //Maintain speeds
    angular = rotMaintain;
    linear = linMaintain;

    // See how mauch distance has been traversed
    float dx = posX - prevX;
    float dy = posY - prevY;
    float displacement = sqrt((dx * dx) + (dy * dy));
    prevX = posX;
    prevY = posY;

    // Find remaining distance to travel
    distanceRemaining = distanceRemaining - displacement;
    if (distanceRemaining <= 0.0) linear = 0; // Stop

    // Monitor rotational displacement
    displacement = std::abs(yaw - prevYaw);
    // Check if displacement is too big (crossing from -pi to +pi)
    // Heading is kept in the range [-pi,+pi] so when turning over pi
    // e.g. -0.98PI to 0.98PI, we need this case
    if (displacement > M_PI) {
        displacement = (M_PI - abs(yaw)) + (M_PI - abs(prevYaw));
    }
    rotationRemaining = rotationRemaining - displacement;
    if (rotationRemaining <= 0.0) angular = 0; // Stop when we've rotated as required 
    prevYaw = yaw;

    rotMaintain = angular;
    linMaintain = linear;

    // Return true if motion completed (no speed)
    bool doneMotion = (angular == 0) && (linear == 0);
    
    if (doneMotion) ROS_DEBUG("Reached destination.");
    else {
        ROS_DEBUG("In motion: D:%.2f S:%.2f | A:%.0f S:%.0f", distanceRemaining,
        linear, RAD2DEG(rotationRemaining), RAD2DEG(angular));
    }
    return doneMotion;
}

void setMotion(double dist, double linSpeed, double rot, double rotSpeed)
{
    // Record new travel requirements
    distanceRemaining = std::abs(dist);
    rotationRemaining = std::abs(rot);

    // Set start point
    prevX = posX;
    prevY = posY;
    prevYaw = yaw;

    // Need to change the global variable 
    linear = linSpeed; 
    angular = rotSpeed;

    // Records speeds to maintain
    rotMaintain = angular;
    linMaintain = linear;

    ROS_DEBUG("Set up motion: D:%.2f S:%.2f | A:%.0f S:%.0f", distanceRemaining, linear,
        RAD2DEG(rotationRemaining), RAD2DEG(angular));
}

bool travel(double dist, double linSpeed, double rot, double rotSpeed) 
{
    static double pd = 0, pls = 0, pr = 0, prs = 0; // Used to store previous state of inputs

    // Check if this is a repeated call
    bool repeatedCall = false;
    if ((dist == pd) && (linSpeed == pls) && (rot == pr) && (rotSpeed == prs)) repeatedCall = true;
    pd = dist;
    pls = linSpeed;
    pr = rot;
    prs = rotSpeed;

    bool doneMotion = false;

    if (repeatedCall == true) {
        // Repeated call, so we're maintaining course
        doneMotion = monitorMotion();
    }
    else {
        // New call, set new course
        setMotion(dist, linSpeed, rot, rotSpeed);
    }

    return doneMotion;
}

bool setHeading(float heading, float speed) 
{
    static float lastHeading = 0, lastSpeed = 0; // Used to monitor parameters 
    static float change = 0, rotVelocity = 0; // Internal parameters to describe the change
    bool doneAlignment = false;

    if ((lastHeading == heading) && (lastSpeed == speed)) {
        // Parameters didn't change, repeat travel() call
        doneAlignment = travel(0, 0, change, rotVelocity);
    }
    else {
        // New parameters, therefore an initial call
        // Record parameters for future reference
        lastHeading = heading;
        lastSpeed = speed;

        // Determine change needed
        float changeForward, changeBackward, tempHeading;

        // Find forward displacement
        if (heading < yaw) tempHeading = heading + 2 * M_PI; // Loop around 
        else tempHeading = heading;
        changeForward = tempHeading - yaw;

        // Find backwards displacement
        if (heading > yaw) tempHeading = heading - 2 * M_PI; // Loop back
        else tempHeading = heading;
        changeBackward = yaw - tempHeading;

        ROS_DEBUG("Going forward %.0f deg, backwards %.0f.", RAD2DEG(changeForward), RAD2DEG(changeBackward));

        // Compare to see which direction is optimal
        if (changeBackward < changeForward) {
            // Going backwards
            rotVelocity = -speed;
            change = changeBackward;
            ROS_INFO("Aligning to heading %.0f deg, currently at %.0f. Change of %.0f right.",
                RAD2DEG(heading), RAD2DEG(yaw), RAD2DEG(change));
        }
        else {
            // Going forwards
            rotVelocity = speed;
            change = changeForward;

            ROS_INFO("Aligning to heading %.0f deg, currently at %.0f. Change of %.0f left.",
                RAD2DEG(heading), RAD2DEG(yaw), RAD2DEG(change));
        }

        

        // Start moving as needed
        doneAlignment = travel(0, 0, change, rotVelocity);
    }

    if (doneAlignment) ROS_INFO("Aligned with new heading.");
    return doneAlignment;
}