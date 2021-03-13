#ifndef BUMPER_HEADER
#define BUMPER_HEADER

#include "globals.h"
#include "movement.h"


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg); // Handles bumber events
void bumperName(u_int8_t index, char output[]); // Used to write which side got hit given the index
void bumperMove(u_int8_t &subState);

#endif