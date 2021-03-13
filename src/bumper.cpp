#include "bumper.h"

// Bumper global variables
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool anyBumperPressed = false;  // Stores overall bumper status
uint8_t lastBumper = 0;         // Stores last bumper pressed

// Retreat constants
const float BUMPER_RETREAT_SPEED = FAST_MOVE;       // Linear retreat speed m/s (must be positive)
const float BUMPER_RETREAT_DIST = 0.1;              // Linear retreat distance
const float BUMPER_RETREAT_ROT = M_PI / 4;          // Angle to twist if a side bumper is struck (rad)
const float BUMPER_RETREAT_ROT_VEL = FAST_SPIN;     // Twist rate (rad/s)

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;

    // Inform user if a bumper is released
    if (msg->state == kobuki_msgs::BumperEvent::RELEASED) {
        char sideName[10];
        bumperName(msg->bumper, sideName);
        ROS_DEBUG("%s bumper (%d) released.", sideName, msg->bumper);
    }
    
    // Example of single bumper monitoring
    // uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT]; 
    // kobuki_msgs::BumperEvent::PRESSED if bumper is pressed, kobuki_msgs::BumperEvent::RELEASED otherwise

    // Record if any bumper is pressed and record that
    anyBumperPressed = false; 
    for (u_int8_t i = 0; i < N_BUMPER; i++) {
        if (bumper[i] == kobuki_msgs::BumperEvent::PRESSED) {
            anyBumperPressed = true;
            
            // Inform user that bumpers are in contact
            char sideName[10];
            bumperName(i, sideName); // Writes which side got hit to sideName
            ROS_WARN("%s bumper impact detected.", sideName);

            state = BUMPER_STATE; // Set robot to bumper state in main loop
            lastBumper = i; // Records which bumper was last struck
        }
    }
}

void bumperName(u_int8_t index, char sideName[]) 
{
switch (index) {
    case 0:
        strcpy(sideName, "Left");
        break;
    case 1:
        strcpy(sideName, "Center");
        break;
    case 2:
        strcpy(sideName, "Right");
        break;
}
}

void bumperMove(u_int8_t &subState)
{
    bool doneMoving = false;

    // Linear retreat from obstacle
    if (subState == 0) {
        doneMoving = travel(BUMPER_RETREAT_DIST, (-BUMPER_RETREAT_SPEED), 0, 0);

        if (doneMoving) {
            subState++;
            ROS_DEBUG("Done backing up from impact.");
        }
    }

    // Set direction based on which bumper was last hit
    if (subState == 1) {
        // Set robot to spin according to last bumper hit (0 is leftmost bumper)
        switch (lastBumper) {
            case 0:
                doneMoving = travel(0,0, BUMPER_RETREAT_ROT, (-BUMPER_RETREAT_ROT_VEL));
                break;
            case 1:
                doneMoving = travel(0, 0, M_PI / 2, BUMPER_RETREAT_ROT_VEL);
                break;
            case 2:
                doneMoving = travel(0, 0, BUMPER_RETREAT_ROT, BUMPER_RETREAT_ROT_VEL);
                break;
        }
    }

    // Wait until done spinning to resume operation
    if (doneMoving) {
        ROS_INFO("Bumper correction complete.");
        state = EXPLORE_STATE;
    }
}