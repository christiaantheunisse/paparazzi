/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
//#include "modules/computer_vision/of_avoidance.h"
#include "of_avoidance.h"
// #include "modules/computer_vision/opencv_example.h"

#include "modules/core/abi.h"
#include "pthread.h"

#include <stdio.h>
// TODO: add ABI broadcast listener and function which responds to a new
// direction index being published by the image processor. In this function,
// run Adam's code to process the index and determine where to go.

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

#ifndef OFF_DIV_SAFE_INDEX
#define OFF_DIV_SAFE_INDEX 1
#endif

static pthread_mutex_t mutex; // Handles the lock of the memory
struct ABI_message_type { // Define struct
    int lowest_detection_index;
    bool new_result;
};
// Shared by video thread and autopilot thread
struct ABI_message_type global_ABI_message; // Make a global var of type ABI_message_type (the custom struct)

// Function
struct image_t *opencv_func(struct image_t *img, uint8_t camera_id);
struct image_t *opencv_func(struct image_t *img, uint8_t camera_id)
{

  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    int lowest_index_tmp = opencv_main((char *) img->buf, img->w, img->h);

      printf("Lowest index: %d\n", lowest_index_tmp);
      pthread_mutex_lock(&mutex);
      global_ABI_message.lowest_detection_index = lowest_index_tmp;
      global_ABI_message.new_result = true;
      pthread_mutex_unlock(&mutex);

  }

  return NULL;
}

void OF_init(void)
{
    pthread_mutex_init(&mutex, NULL);
    global_ABI_message.lowest_detection_index = 0;
    global_ABI_message.new_result = false;

    cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, OPENCVDEMO_FPS, 0);
}

// IMPLEMENT HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ABI broadcast of best direction index
void OF_periodic(void) {
    // Copy the global var to a local var
    struct ABI_message_type local_ABI_message;

    pthread_mutex_lock(&mutex);
    local_ABI_message.new_result = global_ABI_message.new_result;
    local_ABI_message.lowest_detection_index = global_ABI_message.lowest_detection_index;
    pthread_mutex_unlock(&mutex);

    printf("\nIn the periodic funcion (rate = 50Hz):");
    if (local_ABI_message.new_result) {
        printf(" TRUE\n");
        // ABI broadcast
        AbiSendMsgDIVERGENCE_SAFE_HEADING(OFF_DIV_SAFE_INDEX, local_ABI_message.lowest_detection_index);

        pthread_mutex_lock(&mutex);
        global_ABI_message.new_result = false;
        pthread_mutex_unlock(&mutex);

    }

}