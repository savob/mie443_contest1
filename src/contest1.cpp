#include "globals.h"
#include "bumper.h"
#include "laser.h"
#include "movement.h"
#include "scanning.h"
#include "explore.h"

/* State of robot
  0 - Initialization / start
  1 - Exploring
  2 - Bumper collision event
*/
u_int8_t state = SCAN_STATE;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    u_int8_t lastState = 0;         // Used to check if state was changed between loop iterations by the most recent callbacks
    u_int8_t subState = 0;          // Used for states within states
    u_int32_t iterationCount = 0;   // Used to keep track of iterations for timed events like distance traversal

    // Process the callbacks once before entering the loop to kick things off.
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO("STARTING MAIN LOOP");
            
    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce(); // Check callback functions
        iterationCount++;
        bool doneMoving = false;

        ROS_DEBUG("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        
        // Check if state is the same as the last loop
        if (lastState != state) {
            lastState = state;
            subState = 0; 
            iterationCount = 0;
        }
        
        ///////////////////////////////////////////
        // Behaviour states
        // All in one 'if' block so only one is exectuted per loop cycle, even when a change occurs

        if (state == BUMPER_STATE) {
            bumperMove(subState);
        }
        else if (state == EXPLORE_STATE) {
            explore(subState);
        }
        else if (state == SCAN_STATE) {
            scanAroundSpot(subState);
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ROS_DEBUG("Current speeds linear: %.2f m/s angular: %.3f deg/s", linear, RAD2DEG(angular));
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
