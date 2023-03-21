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

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define OBJECT_AVOIDER_VERBOSE TRUE



#define PRINT(string,...) fprintf(stderr, "[object_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef N_DIRBLOCKS
#define N_DIRBLOCKS 5
#endif

#ifndef FOV_ANGLE
#define FOV_ANGLE 80
#endif

#ifndef DIR_CHANGE_THRESHOLD
#define DIR_CHANGE_THRESHOLD 5
#endif

#ifndef OFF_DIV_SAFE_INDEX_ID
#define OFF_DIV_SAFE_INDEX_ID 1
#endif

#ifndef OFF_PAUSE_THREAD_ID
#define OFF_PAUSE_THREAD_ID 2
#endif

int param_DIR_CHANGE_THRESHOLD = DIR_CHANGE_THRESHOLD;

int param_FOV_ANGLE = FOV_ANGLE;
int angle_per_block = param_FOV_ANGLE / N_DIRBLOCKS;
int direction_accumulator[N_DIRBLOCKS] = {0};

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);


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
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
int16_t centerTheshold = 2;
int16_t largeLeft = 0;
int16_t smallLeft = 1;
int16_t smallRight = 3;
int new_heading_index = 0;


const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef OBJECT_AVOIDER_VISUAL_DETECTION_ID
#define OBJECT_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event new_direction_index;
static void 

void update_trajectory_confidence(uint8_t __attribute__((unused)) sender_id, uint8_t __attribute__((unused)) lowest_detection_index) {

  for (int i = 0; i < N_DIRBLOCKS; i++) {

    if (i == lowest_detection_index) {

      direction_accumulator[i]++;

      if (direction_accumulator[i] == param_DIR_CHANGE_THRESHOLD) {

        navigation_state = OBSTACLE_FOUND;
        new_heading_index = lowest_detection_index;

      }

    } else {

      if (direction_accumulator[i] != 0) {

        direction_accumulator[i]--;

      }

    }

  }
  // direction_accumulator[lowest_detection_index]++;

  // // update our safe confidence based on where the lowest image divergence is located 
  // if(lowest_detection_index == centerTheshold){
  //   obstacle_free_confidence -= 2;
  //   obstacle_free_confidence++;
  // } else {
  //   obstacle_free_confidence -= 2; // be more cautious with positive obstacle detections
  // }

  // bound obstacle_free_confidence
  // Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

}



/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void object_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // bind our colorfilter callbacks to receive the color filter outputs

  //CHANGE TO GRAB LOWESTFILTEREDINDEX!!!!!!
  // NOTE 20-3-2023 14:18 - Added Abi message for DIVERGENCE_SAFE_HEADING
  // Just need to 'make' the correct c and h header files (according to what is said
  // in paparazzi tutorial pdf)


  AbiBindMsgDIVERGENCE_SAFE_HEADING(DIVERGENCE_SAFE_HEADING_ID, &new_direction_index, update_trajectory_confidence);
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
  
  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

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
      // stop
      //waypoint_move_here_2d(WP_GOAL);
      //waypoint_move_here_2d(WP_TRAJECTORY);

      // change heading towards the location of lowest optical divergence



      // if (new_heading = largeLeft) {
      //   angle = -20;
      //   heading_increment = -5;

      // } else if (lowestFilteredIndex = smallLeft) {
      //   angle = -10;
      //   heading_increment = -5;

      // } else if (lowestFilteredIndex = smallRight) {
      //   angle = 10;
      //   heading_increment = 5;

      // } else {
      //   angle = 20;
      //   heading_increment = 5;

      // }
      int angle = (1.0 * FOV_ANGLE / N_DIRBLOCKS) * (new_heading_index + 0.5f) - (FOV_ANGLE) / 2.0f;

      AbiSendMsgPAUSE_THREAD(OFF_PAUSE_THREAD, 3);
      increase_nav_heading(angle);

      // navigation_state = SEARCH_FOR_SAFE_HEADING;
      navigation_state = SAFE;

      break;
    case SEARCH_FOR_SAFE_HEADING:
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
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
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
