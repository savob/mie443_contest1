#ifndef SCANNING_HEADER
#define SCANNING_HEADER

#include "globals.h"
/** @name scanAroundSpot
 * @brief Scans around robot current position and select a new heading.
 * @param subState Substate variable from main loop
 **/
void scanAroundSpot(u_int8_t &subState);

/** @name headingDecider
 * @brief Scores and selects headings based on readings taken as robot rotated
 * @param yawVals Yaw values at each step of scan
 * @param minDists Minimum distance recorded by laser scanner at each step
 * @return Absolute heading (rad) that was selected.
 */
float headingDecider(const float yawVals[], const float minDists[]);

#endif