#include "explore.h"

// Deflection allowance when randomly turning
const uint8_t minDeflection = 50;   // Used when we can't go forward
const uint8_t maxDeflection = 135; 

const uint8_t scanChance = 30; // Chance to scan (of 100)

void explore(u_int8_t &subState)
{    
    static float stepToTake, speedToExplore;
    bool doneMoving = false;

    // Bumble about
    if (subState == 0) {
        //Determine step to take
        ROS_INFO("Minimum laser distance is %.2fm.", minLaserDist);
        stepToTake = std::max(0.0f, minLaserDist - WALL_CLEARANCE); //Used to avoid negative is already too close
        stepToTake = std::min(EXPLORE_STEP, stepToTake);

        // See if we'll stay out of the slow zone or not
        if ((minLaserDist - stepToTake) < SLOW_DOWN_DIST) speedToExplore = SLOW_MOVE;
        else speedToExplore = FAST_MOVE;

        ROS_INFO("Exploring. Going forward %.2fm at a speed of %.2f m/s.", stepToTake, speedToExplore);
        subState++;
    }

    // Do the bumbling about
    if (subState == 1) {
        doneMoving = travel(stepToTake, speedToExplore, 0, 0);

        // If done bumbling about, randomly decide between a random turn or a scan
        if (doneMoving == true) {
            uint8_t diceRoll = rand() % 100; // "!rtd" at ctf_2fort

            if (diceRoll < scanChance) state = SCAN_STATE; // Do a proper scan
            else {
                // Determine how much to spin, up to a set max to not go backwards
                // If we're close to the wall, we want the spin to pass a minimum to go away
                if (minLaserDist < (SLOW_DOWN_DIST / 2.0)) {
                    stepToTake = DEG2RAD(centeredAngle(minDeflection, maxDeflection));
                }
                else stepToTake = DEG2RAD(centeredAngle(0, maxDeflection));

                // 50/50 to going left or right
                if ((rand() % 2) == 1) speedToExplore = FAST_SPIN;
                else speedToExplore = (-FAST_SPIN);

                ROS_INFO("Turning randomly. %.0f degrees, at a rate of %.0f deg/s.", RAD2DEG(stepToTake), RAD2DEG(speedToExplore));
                subState++;
            }
        }
    }

    // Spin to new position and bumble again
    if (subState == 2) {
        doneMoving = travel(0, 0, stepToTake, speedToExplore);
        if (doneMoving) subState++;
    }

    if (subState == 3) {
        // Check if this direction is open
        if (minLaserDist < (SLOW_DOWN_DIST / 2.0)) {
            // Not open, swing around to opposite side
            subState++;
            stepToTake = 2 * stepToTake;
            speedToExplore = -speedToExplore;
        }
        else subState = 0;
    }

    // Swing around to other side
    if (subState == 4) {
        doneMoving = travel(0, 0, stepToTake, speedToExplore);
        if (doneMoving) subState = 0;
    }
}

float centeredAngle(float low, float high)
{
    // Find the range we're allowed
    float range = high - low;

    // Gets a random number in the range
    float angle = rand() % int(range);
    
    // Recalculate new angle, weighed towards 0 (to keep the robot going centered)
    const float weight = 3.0;  // Weighing power, the higher this is the tighter around 0 the results
    angle = pow(angle, weight) * (range / pow(range, weight));

    // Add offset for the start of the range
    angle = angle + low;

    return angle;
}