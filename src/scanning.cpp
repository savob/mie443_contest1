#include "scanning.h"
#include "movement.h"

// File globals
const uint8_t NUMBER_OF_STEPS = 12; // Number of stops taken around the circle
const float STEP_INCREMENT = 2 * M_PI / NUMBER_OF_STEPS;

const float SCAN_SPEED = FAST_SPIN;
const float ALIGN_SPEED = FAST_SPIN;

void scanAroundSpot(u_int8_t &subState) {
    static float yawValues[NUMBER_OF_STEPS];
    static float minDistances[NUMBER_OF_STEPS];
    static float targetHeading;

    static uint8_t stepsCompleted;
    bool movementDone = false;

    // Reset for each spin
    if (subState == 0) {
        stepsCompleted = 0;
        subState++;
        ROS_INFO("Performing 360-degree scan.");
    }

    // Rotating
    if (subState == 1) {
        movementDone = travel(0, 0, STEP_INCREMENT, SCAN_SPEED);

        if (movementDone) {
            // Record parameters for this step
            yawValues[stepsCompleted] = yaw;
            minDistances[stepsCompleted] = minLaserDist;

            ROS_INFO("Step %d. Distance: %.2f, Yaw: %.0f.", stepsCompleted, minLaserDist, RAD2DEG(yaw));

            stepsCompleted++;

            // Check if the robot needs to do more steps
            if (stepsCompleted < NUMBER_OF_STEPS) travel(0, 0, 0, 0); // Clears old travel instruction (so we can repeat the same step one)
            else {
                // Find ideal heading and head that way for the next substep
                targetHeading = headingDecider(yawValues, minDistances);
                subState++;
                ROS_INFO("Scan completed, heading selected %.0f.", RAD2DEG(targetHeading));
            }
        }
    }

    // Align to heading to take
    if (subState == 2) {
        movementDone = setHeading(targetHeading, ALIGN_SPEED);

        if (movementDone) state = EXPLORE_STATE;
    }
}

float headingDecider(const float yawVals[], const float minDists[]) {
    float decidedHeading;
    // Calculate a score for each step
    float scores[NUMBER_OF_STEPS];
    float totalScore = 0;
    uint8_t selectedIndex = 0;
    
    for (uint8_t i = 0; i < NUMBER_OF_STEPS; i++) {
        scores[i] = minDists[i];

        totalScore = scores[i] + totalScore;
    }

    if (totalScore == 0) {
        ROS_ERROR("Laser scan came back with only 0's! Going straight.");
        return yaw;
    }

    // Weighted random for openness
    // Selects a random number from 0 to the cumulative scores for a scan. 
    // Whichever scan step "buckets" this value is selected
    float selectedValue = 0;
    float accumulatedScore = 0;

    // Gets the selected value
    int32_t maxValue = int(floor(totalScore * 100.0)); // Get upper limit of random value

    selectedValue = rand() % maxValue;
    selectedValue = selectedValue / 100.0;

    for (uint8_t i = 0; i < NUMBER_OF_STEPS; i++) {
        accumulatedScore = scores[i] + accumulatedScore;

        // Once the value is "bucketed" exit
        if (accumulatedScore > selectedValue) {
            selectedIndex = i;
            break;
        }
    }
    ROS_INFO("Total score %.2f, selected value %.2f, index %d.", totalScore, selectedValue, selectedIndex);

    /*
    // Just going for max
    for (uint8_t i = 1; i < NUMBER_OF_STEPS; i++) {
        if (scores[i] > scores[selectedIndex]) selectedIndex = i; // Update to new max
    }
    
    */
    decidedHeading = yawVals[selectedIndex];
    return decidedHeading; // Return ideal yaw
}