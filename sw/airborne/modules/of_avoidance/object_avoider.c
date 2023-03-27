/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <sys/time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define OBJECT_AVOIDER_VERBOSE TRUE



#define PRINT(string,...) fprintf(stderr, "[object_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static void lowest_index_cb(uint8_t sender_id, uint8_t i_safe);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};


// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t obstacle_dist = 0;                // Distance to closest object
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float angle = 0;
float heading_increment = 100.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
int16_t centerTheshold = 2;
int16_t largeLeft = 0;
int16_t smallLeft = 1;
int16_t smallRight = 3;
int new_heading_index = 0;

#ifndef N_DIRBLOCKS
#define N_DIRBLOCKS 5
#endif

#ifndef FOV_ANGLE
#define FOV_ANGLE 160
#endif

#ifndef DIR_CHANGE_THRESHOLD
#define DIR_CHANGE_THRESHOLD 10
#endif

#ifndef PAUSE_TIME
#define PAUSE_TIME 15
#endif

unsigned long pause_time = PAUSE_TIME;
int param_DIR_CHANGE_THRESHOLD = DIR_CHANGE_THRESHOLD;
int param_FOV_ANGLE = FOV_ANGLE;
int param_N_DIRBLOCKS = N_DIRBLOCKS;
int direction_accumulator[N_DIRBLOCKS] = {0};
int center_index = N_DIRBLOCKS / 2;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

// Pause vars
bool do_pause = false;
struct timeval start_time, now;
//clock_t start_time, now;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */

static abi_event lowestFilteredIndex;
// Global var with lowest index
uint8_t lowest_index = 120;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ABI DIVERGENCE_SAFE_HEADING callback
// Callback should always have `uint8_t sender_id`
static void lowest_index_cb(uint8_t sender_id, uint8_t i_safe) {
//    printf("ABI receives new index: %d", i_safe);
    printf("Pause: %d", do_pause);
    if (!do_pause) {
        for (int i = 0; i < N_DIRBLOCKS; i++) {
            if (i == i_safe) {
                direction_accumulator[i]++;
                if (direction_accumulator[i] == param_DIR_CHANGE_THRESHOLD && i != center_index) {
                    navigation_state = OBSTACLE_FOUND;
                    new_heading_index = i_safe;
                }
            } else {
                if (direction_accumulator[i] != 0) {
                    direction_accumulator[i]--;
                }

            }
        }
    } else {
        gettimeofday(&now, NULL);
//        now = clock();
        unsigned long difference = (now.tv_sec - start_time.tv_sec) * 1000000 + now.tv_usec - start_time.tv_usec;
        printf("Difference: %lu us", difference);

        if ((difference) >= (pause_time * 100 * 1000)) { // * 100 ms * 1000 (us -> ms)
//        if ((double)(now - start_time)/CLOCKS_PER_SEC >= PAUSE_TIME) {
            printf("STOP PAUSING ==========================");
            do_pause = false;
            for (int i = 0; i < N_DIRBLOCKS; i++) {
                direction_accumulator[i] = 0;
            }
        }
    }
}


/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void object_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ABI_BROADCAST = 255 = receive messages from all senders on DIVERGENCE_SAFE_HEADING
  AbiBindMsgDIVERGENCE_SAFE_HEADING(ABI_BROADCAST, &lowestFilteredIndex, lowest_index_cb);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void object_avoider_periodic(void)
{
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }

//    float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
    float moveDistance = fminf(maxDistance, 0.5f);
    int angle;

    switch (navigation_state){
        case SAFE:
            // Move waypoint forward
            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = OUT_OF_BOUNDS;
            } else {
                moveWaypointForward(WP_GOAL, moveDistance);
            }

            break;
        case OBSTACLE_FOUND:
            angle = (1.0 * param_FOV_ANGLE / param_N_DIRBLOCKS) * (new_heading_index + 0.5f) - (param_FOV_ANGLE) / 2.0f;

            // Clear the detections for `int` * 100 ms
            do_pause = true;
            gettimeofday(&start_time, NULL);
//            start_time = clock();

            increase_nav_heading(angle);

            // navigation_state = SEARCH_FOR_SAFE_HEADING;
            navigation_state = SAFE;

            break;
        case SEARCH_FOR_SAFE_HEADING: // Currently not used
            increase_nav_heading(heading_increment);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                navigation_state = SAFE;
            }
            break;
        case OUT_OF_BOUNDS:
            increase_nav_heading(heading_increment);
            moveWaypointForward(WP_TRAJECTORY, 1.5f);

            if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                // add offset to head back into arena
                increase_nav_heading(heading_increment);

                // reset safe counter
                obstacle_free_confidence = 0;

                // ensure direction is safe before continuing
                navigation_state = SAFE;
            }

            // Clear the detections for `int` * 100 ms
            do_pause = true;
            gettimeofday(&start_time, NULL);
//            start_time = clock();
            break;
        default:
            break;
    }

    return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
