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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_optical_flow.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include <algorithm>
#include <stdlib.h>
#include "pthread.h"

#ifndef DET_THRESHOLD
#define DET_THRESHOLD 0.2
#endif

double param_DET_THRESHOLD = DET_THRESHOLD;

#ifndef RESIZE_FX
#define RESIZE_FX 0.5
#endif
#ifndef RESIZE_FY
#define RESIZE_FY 0.5
#endif

double param_RESIZE_FX = RESIZE_FX;
double param_RESIZE_FY = RESIZE_FY;

#ifndef OFF_PYR_SCALE
#define OFF_PYR_SCALE 0.5
#endif
#ifndef OFF_LEVELS
#define OFF_LEVELS 3
#endif
#ifndef OFF_WINSIZE
#define OFF_WINSIZE 10
#endif
#ifndef OFF_ITERATIONS
#define OFF_ITERATIONS 3
#endif
#ifndef OFF_POLY_N
#define OFF_POLY_N 1
#endif
#ifndef OFF_POLY_SIGMA
#define OFF_POLY_SIGMA 1.2
#endif
#ifndef OFF_FLAGS
#define OFF_FLAGS 0
#endif

double param_OFF_PYR_SCALE = OFF_PYR_SCALE;
int param_OFF_LEVELS = OFF_LEVELS;
int param_OFF_WINSIZE = OFF_WINSIZE;
int param_OFF_ITERATIONS = OFF_ITERATIONS;
int param_OFF_POLY_N = OFF_POLY_N;
double param_OFF_POLY_SIGMA = OFF_POLY_SIGMA;
double param_OFF_FLAGS = OFF_FLAGS;

#ifndef CROP_X
#define CROP_X 0
#endif
#ifndef CROP_Y
#define CROP_Y 25
#endif
#ifndef CROP_WIDTH
#define CROP_WIDTH 320
#endif
#ifndef CROP_HEIGHT
#define CROP_HEIGHT 125
#endif

int param_CROP_X = CROP_X;
int param_CROP_Y = CROP_Y;
int param_CROP_WIDTH = CROP_WIDTH;
int param_CROP_HEIGHT = CROP_HEIGHT;

#ifndef N_DIRBLOCKS
#define N_DIRBLOCKS 5
#endif
#ifndef M_DIRFILTER
#define M_DIRFILTER 1
#endif

int param_N_DIRBLOCKS = N_DIRBLOCKS;

#ifndef PERCENTAGE_HISTORY_IMPORTANCE
#define PERCENTAGE_HISTORY_IMPORTANCE 0.7
#endif

double param_PERCENTAGE_HISTORY_IMPORTANCE = PERCENTAGE_HISTORY_IMPORTANCE;

#ifndef PREFERRED_INDEX
#define PREFERRED_INDEX 2
#endif
#ifndef SIDE_PENALTY
#define SIDE_PENALTY 0.05
#endif

int param_PREFERRED_INDEX = PREFERRED_INDEX;
double param_SIDE_PENALTY = SIDE_PENALTY;

#ifndef OFF_DIV_SAFE_INDEX_ID
#define OFF_DIV_SAFE_INDEX_ID 1
#endif

#ifndef OFF_PAUSE_THREAD_ID
#define OFF_PAUSE_THREAD_ID 2
#endif

Mat old_frame_grayscale;
double detection_history[N_DIRBLOCKS] = {};
int lowest_detection_index = 0;
bool new_result = false;
bool do_pause = false;
int pause_dura = 0;

int find_best_direction_index(const cv::Mat &detection_horizon) {

  // Initialise array to hold values of detection horizon
  std::vector<float> array;

  // Just to be safe, check if Mat is continuous
  if (detection_horizon.isContinuous())

    // Assign Mat to array, mapping all values to it
    array.assign((float*)detection_horizon.data, (float*)detection_horizon.data + detection_horizon.total() * detection_horizon.channels());

  // Determine pixel width of detection horizon
  int horizon_width = detection_horizon.cols;

  // Determine the amount of cells that it skips per block
  int pixels_per_block = horizon_width / N_DIRBLOCKS;

  double detections_per_block[N_DIRBLOCKS] = {};
  double updated_detection_history[N_DIRBLOCKS] = {};

  // Loop through discrete direction blocks
  for (int n = 0; n < N_DIRBLOCKS; n++) {

      // Determine low and high index of block
      int lowest_index = n * pixels_per_block;
      int highest_index = std::min((n + 1) * pixels_per_block, horizon_width) - 1;

      // Variable to accumulate sum of detections
      double detection_count = 0;

      // Loop through indices belonging to this block
      for (int i = lowest_index; i <= highest_index; i++) {

          // Accumulate sum of detections in that block
          detection_count += 1 - array[i];

      }

      // Set detection count for this block to the sum of detections
      detections_per_block[n] = detection_count;

  }

  // Loop through direction blocks again
  for (int i = 0; i < N_DIRBLOCKS; i++) {

      // Variable to store filtered (convoluted) detections
      double filtered_detection_sum = 0;

      // Loop through convolution filter
      for (int j = -M_DIRFILTER; j <= M_DIRFILTER; j++) {

          // Check if the index is not out of bounds
          if (i + j >= 0 && i + j < N_DIRBLOCKS) {

              // Add the 'detection score' to the total; weigh the neighbours to be worth less
              filtered_detection_sum += (1.0 / (std::abs(j) + 1.0)) * detections_per_block[i + j];

          }

      }

      filtered_detection_sum += std::abs(i - param_PREFERRED_INDEX) * param_SIDE_PENALTY;

      // Weigh the detections and take historical detections into account as well
      updated_detection_history[i] = (1 - param_PERCENTAGE_HISTORY_IMPORTANCE) * filtered_detection_sum + param_PERCENTAGE_HISTORY_IMPORTANCE * detection_history[i];

  }

  // Copy updated detection history to the history array
  std::copy(std::begin(updated_detection_history), std::end(updated_detection_history), std::begin(detection_history));

  // Variables to hold 'best' index and score
  double lowest_filtered_score = 10000;
  int lowest_filtered_index = -1;

  // Loop through direction blocks ahain
  for (int i = 0; i < N_DIRBLOCKS; i++) {

      // If it finds a better score...
      if (detection_history[i] < lowest_filtered_score) {

          // Set to this newest best index and score
          lowest_filtered_score = detection_history[i];
          lowest_filtered_index = i;

      }
  }

  return lowest_filtered_index;
}

cv::Mat calculate_divergence(const cv::Mat& flow)
{
  // Split the two channels of the flow Mat into separate Mats
  std::vector<cv::Mat> flow_channels;
  cv::split(flow, flow_channels);
  cv::Mat flow_x = flow_channels[0];
  cv::Mat flow_y = flow_channels[1];

  // Initialize output Mat with zeros
  cv::Mat divergence = cv::Mat::zeros(flow_x.size(), CV_32FC1);

  // Create Mat to hold x and y derivatives of flow
  cv::Mat dx, dy, dx_abs, dy_abs;

  // Create kernels for the center-difference in x and y direction
  Mat kernelx = (Mat_<float>(1, 3) << -0.5, 0, 0.5);
  Mat kernely = (Mat_<float>(3, 1) << -0.5, 0, 0.5);

  // Apply kernels and get result
  cv::filter2D(flow_x, dx, -1, kernelx);
  cv::filter2D(flow_y, dy, -1, kernely);

  dx_abs = cv::abs(dx);
  dy_abs = cv::abs(dy);

  // Calculate divergence over x and y axis
  cv::Mat div_x = dx_abs + dy_abs;

  // Store the computed divergence in the output Mat
  div_x.copyTo(divergence);

  return divergence;
}

void opencv_main(char *img, int width, int height) {

  bool local_do_pause = false;
  int local_pause_dura = 0;

  pthread_mutex_lock(&mutex);
  local_do_pause = do_pause;
  local_pause_dura = pause_dura;
  pthread_mutex_unlock(&mutex);

  if (local_do_pause) {

    pthread_mutex_lock(&mutex);
    do_pause = false
    pthread_mutex_unlock(&mutex);
  
    std::chrono::seconds dura(local_pause_dura);
    std::this_thread::sleep_for(dura);

    for (int n = 0; n < N_DIRBLOCKS; n++) {

      detection_history[n] = 0;

    }

  }


  // Cast the image struct into an opencv Mat
  Mat frame(height, width, CV_8UC2, img);

  // Initialize Mat to hold grayscale version of image
  Mat frame_grayscale, gray_resized;

  // Convert to grayscale and put in previously defined Mat
  cv::cvtColor(frame, frame_grayscale, CV_YUV2GRAY_Y422);

  // Resize gray frame
  cv::resize(frame_grayscale, gray_resized, Size(), param_RESIZE_FX, param_RESIZE_FY);

  // Define cropped region (x, y, width, height)
  cv::Rect cropped_region(param_CROP_X, param_CROP_Y, param_CROP_WIDTH, param_CROP_HEIGHT);

  // Crop frame to smaller region
  Mat cropped_gray_frame = gray_resized(cropped_region);

  // If there is no previous frame (i.e. the drone just started up)
  // set this frame to be old frame also
  if (old_frame_grayscale.empty()) {

      cropped_gray_frame.copyTo(old_frame_grayscale);

  }

  // Create Mat to hold flow field
  Mat flow_field;

  // Calculate optical flow using old gray frame and new gray frame. Store in flowUmat
  // prev, next, src, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags
  cv::calcOpticalFlowFarneback(old_frame_grayscale, cropped_gray_frame, flow_field, param_OFF_PYR_SCALE, param_OFF_LEVELS, param_OFF_WINSIZE, param_OFF_ITERATIONS, param_OFF_POLY_N, param_OFF_POLY_SIGMA, param_OFF_FLAGS);

  // Copy this frame to the oldGray Mat, to be used in next frame
  cropped_gray_frame.copyTo(old_frame_grayscale);

  // Initialise output Mat for divergence calculation results
  Mat output;

  // Calculate divergence from flow
  output = calculate_divergence(flow_field);

  // Mat to hold the mean for each image column
  Mat column_mean;

  // Take mean of each column
  cv::reduce(output, column_mean, 0, cv::REDUCE_AVG);

  // Mat to hold the thresholded 1-D mean divergence array
  Mat thresholded_divergence;

  // Threshold it;
  // inverse means below threshold is set to 1; otherwise to 0
  // src, dst, threshold, max_value, method
  cv::threshold(column_mean, thresholded_divergence, param_DET_THRESHOLD, 1, THRESH_BINARY_INV);

  int local_lowest_detection_index = find_best_direction_index(thresholded_divergence);

  pthread_mutex_lock(&mutex);
  lowest_detection_index = local_lowest_detection_index;
  new_result = true;
  pthread_mutex_unlock(&mutex);

}




// BELOW IS CODE FROM THE OLD EXAMPLE FILE
// LEFT IT HERE FOR INSPIRATION. REMOVE LATER.
int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

#if OPENCVDEMO_GRAYSCALE
  //  Grayscale image example
  cvtColor(M, image, CV_YUV2GRAY_Y422);
  // Canny edges, only works with grayscale image
  int edgeThresh = 35;
  Canny(image, image, edgeThresh, edgeThresh * 3);
  // Convert back to YUV422, and put it in place of the original image
  grayscale_opencv_to_yuv422(image, img, width, height);
#else // OPENCVDEMO_GRAYSCALE
  // Color image example
  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);
#endif // OPENCVDEMO_GRAYSCALE

  return 0;
}

void pause_detection_cb(uint8_t __attribute__((unused)) sender_id, uint8_t __attribute__((unused)) local_pause_dura) {

  pthread_mutex_lock(&mutex);
  do_pause = true;
  pause_dura = local_pause_dura;
  pthread_mutex_unlock(&mutex);

}

void OF_init(void) {

  cv_add_to_device(&OFF_CAMERA, opencv_main, OFF_FPS, 0);

  AbiBindMsgPAUSE_THREAD(PAUSE_THREAD_ID, &new_pause_detection, pause_detection_cb);

}

void OF_periodic(void) {

  bool local_new_result;
  int local_lowest_detection_index;

  pthread_mutex_lock(&mutex);
  local_new_result = new_result;
  local_lowest_detection_index = lowest_detection_index;
  pthread_mutex_unlock(&mutex);

  if (local_new_result) {
    // IMPLEMENT HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // ABI broadcast of best direction index
    AbiSendMsgDIVERGENCE_SAFE_HEADING(OFF_DIV_SAFE_INDEX, local_lowest_detection_index);

    pthread_mutex_lock(&mutex);
    new_result = false;
    pthread_mutex_unlock(&mutex);

  }

}