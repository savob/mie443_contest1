#include "laser.h"

// Laser related values
float minLaserDist = 0;         // Minimum distance picked up but most recent laser scan
int32_t nLasers = 0;            // Number of laser beams available
int32_t desiredNLasers = 0;     // Number of lasers we will store given our desired range
int32_t desiredAngle = 10;      // Desired view range around center axis, in degrees
float aveLaserDist = 0;         // Average laser distance on last scan

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Set globals to defaults
    minLaserDist = std::numeric_limits<float>::infinity(); 
    aveLaserDist = 0;

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    //ROS_INFO("Width of laser scan array %.3f to %.3f", RAD2DEG(msg->angle_min), RAD2DEG(msg->angle_max));

    // Find minimum distance in selected FOV range
    // Set up loop limits based on our desired range
    uint32_t startIndex = 0, endIndex = nLasers; // Default to max range
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        // If narrorer than allowed range
        startIndex =  nLasers / 2 - desiredNLasers; 
        endIndex = nLasers / 2 + desiredNLasers;
    }
    
    // Loop through finding minimum and adding non-infinite values to get an average
    for (uint32_t laser_idx = startIndex; laser_idx < endIndex; ++laser_idx) {
        float reading = msg->ranges[laser_idx];
        minLaserDist = std::min(minLaserDist, reading);
        if ((reading > 0) && (reading != std::numeric_limits<float>::infinity())) {
            aveLaserDist = aveLaserDist + reading;
        }
    }

    aveLaserDist = aveLaserDist / float(nLasers); // Divide sum to get average

    // The laser scanner we use has a lower limit of 0.45m, anthing closer is registered as infinity.
    // So if the minimum recording is 0.45m or inf, we treat it as a contact
    if (minLaserDist < 0.46) minLaserDist = 0; 
    if (minLaserDist == std::numeric_limits<float>::infinity()) minLaserDist = 0;

    ROS_DEBUG("Laser scan complete. Min: %.2fm, Average: %.2fm.", minLaserDist, aveLaserDist);
}