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
#include <stdio.h>
#include <cstdint>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;
#include "opencv_image_functions.h"
#include <algorithm>
#include <stdlib.h>
#include <chrono>
#include <thread>

#ifndef DET_THRESHOLD
#define DET_THRESHOLD 0.2
#endif

double param_DET_THRESHOLD = DET_THRESHOLD;

#ifndef RESIZE
#define RESIZE 0.25
#endif

#ifndef RESIZE_FX
#define RESIZE_FX RESIZE
#endif
#ifndef RESIZE_FY
#define RESIZE_FY RESIZE
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
#define OFF_ITERATIONS 1
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

// Current values: Use width [60, 460] out of 520 px and for the height [50, 100] out of 240 px
// Total [width, height] = [520, 240]

#ifndef CROP_X
#define CROP_X 60
#endif
#ifndef CROP_Y
#define CROP_Y 50
#endif
#ifndef CROP_HEIGHT
#define CROP_HEIGHT 50
#endif

int param_CROP_X = CROP_X * RESIZE_FX;
int param_CROP_Y = CROP_Y  * RESIZE_FY;  // 50 * 0.25
int param_CROP_WIDTH = 520 * RESIZE_FX -  2 * param_CROP_X; // (520 * 0.25) - 2 * 15 = 100
int param_CROP_HEIGHT = CROP_HEIGHT * RESIZE_FY;  // Total 240 -> [50, 100]

#ifndef N_DIRBLOCKS
#define N_DIRBLOCKS 5
#endif
#ifndef M_DIRFILTER
#define M_DIRFILTER 1
#endif

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

Mat old_frame_grayscale;
double detection_history[N_DIRBLOCKS] = {};

int start_time;
int pause_duration;
bool pausing;
int center = N_DIRBLOCKS / 2;

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

int opencv_main(char *img, int width, int height, bool do_pause, int pause_dura) {


    if (do_pause) {
        int now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        start_time = now;
        pause_duration = pause_dura * 100; // pause_dura per 100 ms
        pausing = true;
        for (int n = 0; n < N_DIRBLOCKS; n++) {
            detection_history[n] = 0;
        }
        Mat empty_mat;
        old_frame_grayscale = empty_mat;
    }

    if (pausing) {
        int now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (((now - start_time) >= pause_duration)) {
            pausing = false;
        }

        return center;
    } else {



        // Cast the image struct into an opencv Mat
        Mat frame(height, width, CV_8UC2, img);
        // Rotate
//        cv::Point2f center(frame.cols / 2., frame.cols / 2.);
//        cv::Mat r = cv::getRotationMatrix2D(center, 90, 1.0);
//        cv::warpAffine(frame, frame, r, cv::Size(frame.rows, frame.cols));
        cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);

        // Initialize Mat to hold grayscale version of image
        Mat frame_grayscale, gray_resized;


        // Convert to grayscale and put in previously defined Mat
        cv::cvtColor(frame, frame_grayscale, CV_YUV2GRAY_Y422);

        // DEBUG STEP
        // grayscale_opencv_to_yuv422(frame_grayscale, img, width, height);
        //   printf("frame_grayscale:\n\theight: %d\n\twidth: %d\n", frame_grayscale.size().height, frame_grayscale.size().width);
        // height: 520, width: 240

        // Resize gray frame
        cv::resize(frame_grayscale, gray_resized, Size(), param_RESIZE_FX, param_RESIZE_FY);
        //  printf("gray_resized:\n\theight: %d\n\twidth: %d\n", gray_resized.size().height, gray_resized.size().width);
        // height: 260, width: 120 [param_RESIZE_FX = 0.5, param_RESIZE_FY = 0.5]

        // Define cropped region (x, y, width, height)
//        printf("CROP_X, CROP_Y, CROP_WIDTH, CROP_HEIGHT: (%d, %d, %d, %d)", param_CROP_X, param_CROP_Y, param_CROP_WIDTH, param_CROP_HEIGHT);
        cv::Rect cropped_region(param_CROP_X, param_CROP_Y, param_CROP_WIDTH, param_CROP_HEIGHT);
        //  printf("cropped_region:\n\theight: %d\n\twidth: %d\n", cropped_region.height, cropped_region.width);
        // current height: 260, width: 50 [CROP_X = 58, CROP_Y = 0, CROP_WIDTH = 50, CROP_HEIGHT = 260]

        // Crop frame to smaller region
        Mat cropped_gray_frame = gray_resized(cropped_region);
        //  printf("cropped_gray_frame:\n\theight: %d\n\twidth: %d\n", cropped_gray_frame.size().height, cropped_gray_frame.size().width);
        // current height: 260, width: 50

        // DEBUG STEP -> Does not work because the video expects a size of 240 x 520
        //  grayscale_opencv_to_yuv422(cropped_gray_frame, img, width, height);

        // If there is no previous frame (i.e. the drone just started up)
        // set this frame to be old frame also
        if (old_frame_grayscale.empty()) {

            cropped_gray_frame.copyTo(old_frame_grayscale);

        }

        // Create Mat to hold flow field
        Mat flow_field;

        // Calculate optical flow using old gray frame and new gray frame. Store in flowUmat
        // prev, next, src, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags

        // FOR TIMING
//        auto start = std::chrono::high_resolution_clock::now();


        cv::calcOpticalFlowFarneback(old_frame_grayscale, cropped_gray_frame, flow_field, param_OFF_PYR_SCALE,
                                     param_OFF_LEVELS, param_OFF_WINSIZE, param_OFF_ITERATIONS, param_OFF_POLY_N,
                                     param_OFF_POLY_SIGMA, param_OFF_FLAGS);

        // FOR TIMING
//        auto stop = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//        printf("Duration of rotation OF: %d\n", duration);


        //  printf("Flow_field:\nheight: %d\nwidth: %d\nchannels: %d\n", debug_FF.rows, debug_FF.cols, debug_FF.channels());

        // visualization
        //    Mat flow_parts[2];
        //    split(flow_field, flow_parts);
        //    printf("\nOF: ");
        //    for(int i=0; i<flow_field.cols; i++)
        //        for(int j=0; j<flow_field.rows; j++)
        //            printf(" %f", flow_field.at<float>(j, i));
        //    printf("\n\n\n");

        //    Mat magnitude, angle, magn_norm;
        //    cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        //    normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        //    angle *= ((1.f / 360.f) * (180.f / 255.f));
        //    //build hsv image
        //    Mat _hsv[3], hsv, hsv8, bgr;
        //    _hsv[0] = angle;
        //    _hsv[1] = Mat::ones(angle.size(), CV_32F);
        //    _hsv[2] = magn_norm;
        //    merge(_hsv, 3, hsv);
        //    hsv.convertTo(hsv8, CV_8U, 255.0);
        //    cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        //    imshow("frame2", bgr);
        //    int keyboard = waitKey(30);
        //    if (keyboard == 'q' || keyboard == 27)
        //        break;
        //    prvs = next;


        // Copy this frame to the oldGray Mat, to be used in next frame
        cropped_gray_frame.copyTo(old_frame_grayscale);

        // Initialise output Mat for divergence calculation results
        Mat output;

        // Calculate divergence from flow
        output = calculate_divergence(flow_field);
        //    printf("Divergence: h = %d, w = %d, c = %d", output.rows, output.cols, output.channels());

        // Mat to hold the mean for each image column
        Mat column_mean;

        // Take mean of each column
        cv::reduce(output, column_mean, 0, cv::REDUCE_AVG);



        // Mat to hold the thresholded 1-D mean divergence array
        Mat thresholded_divergence;

        //    printf("\nDivergence: ");
        //    for(int i=0; i<column_mean.cols; i++)
        //        printf(" %d", column_mean.at<float>(0, i));
        //    printf("\n\n\n");

        // Threshold it;
        // inverse means below threshold is set to 1; otherwise to 0
        // src, dst, threshold, max_value, method
        cv::threshold(column_mean, thresholded_divergence, param_DET_THRESHOLD, 1, THRESH_BINARY_INV);
        //  printf("Thresholded div:\nheight: %d\nwidth: %d\nchannels: %d\n", thresholded_divergence.rows, thresholded_divergence.cols, thresholded_divergence.channels());

        //    printf("Columnmean: h = %d, w = %d, c = %d", column_mean.rows, column_mean.cols, column_mean.channels());
        //
        //    printf("\nOF: ");
        //    for(int i=0; i<thresholded_divergence.cols; i++)
        //        for(int j=0; j<thresholded_divergence.rows; j++)
        //            printf(" %f", thresholded_divergence.at<float>(j, i));
        //    printf("\n\n\n");
        // Display the divergence in the bottom of the image

        // Size up to an bigger image
        Mat divergence_img;


        cv::resize(thresholded_divergence, divergence_img, Size(520, 50));
        //    int size_1[3] = {divergence_img.rows, divergence_img.cols, 1};
        //    Mat divergence_img_;
        //    cv::resize(divergence_img, divergence_img_, cv::Size(520, 50, ))

        //    Mat divergence_img_zeros = Mat::zeros(divergence_img.rows, divergence_img.cols, divergence_img.type());
        //    Mat divergence_img_out;
        //    Mat in[2] = { divergence_img,divergence_img_zeros};
        //    merge(in, 2, divergence_img_out);
        //    printf("div img_ (h, w, c, d): %d, %d, %d, %d\n", divergence_img_out.rows, divergence_img_out.cols, divergence_img_out.channels(), divergence_img_out.dims);

        //    std::vector<cv::Mat> vChannels;
        //    for (unsigned int c = 0; c < 2; c++)
        //    {
        //        vChannels.push_back(divergence_img);
        //    }
        //    cv::Mat divergence_img_2;
        //    cv::merge(vChannels, divergence_img_2);


        //  Mat divergence_img_(3, {divergence_img.rows, divergence_img.cols, 1}, divergence_img.type(), divergence_img.data);
        //  cv::cvtColor(divergence_img, divergence_img, CV_GRAY2YUV);
        //  cv::cvtColor(divergence_img, divergence_img, CV_YUV2BGR_Y422);
        //    divergence_img_out.convertTo(divergence_img_out, CV_8U);
        //    for(int i=0; i<divergence_img_out.cols; i++)
        //        for(int j=0; j<divergence_img_out.rows; j++)
        //            printf(" (%d %d)", divergence_img_out.at<uchar>(j, i, 0), divergence_img_out.at<uchar>(j, i, 1));
        //    printf("\n\n\n");

        //  printf("div img (h, w, d): %d, %d, %d\n", divergence_img_out.rows, divergence_img_out.cols, divergence_img_out.channels());
        //  printf("frame (h, w, d): %d, %d, %d\n\n", frame.rows, frame.cols, frame.channels());
        //    divergence_img_out.copyTo(frame(cv::Rect(0, 190, divergence_img_out.cols, divergence_img_out.rows)));
        //    printf("COPIED");
        //    printf("frame after insert (h, w, d): %d, %d, %d\n\n", frame.rows, frame.cols, frame.channels());

        divergence_img = divergence_img * 255;
        //    printf("div before insert (h, w, d): %d, %d, %d\n\n", divergence_img.rows, divergence_img.cols, divergence_img.channels());
        //    printf("frame before insert (h, w, d): %d, %d, %d\n\n", frame_grayscale.rows, frame_grayscale.cols, frame_grayscale.channels());
        divergence_img.convertTo(divergence_img, CV_8U);
        divergence_img.copyTo(frame_grayscale(cv::Rect(0, 190, divergence_img.cols, divergence_img.rows)));
        //    printf("frame after insert (h, w, d): %d, %d, %d\n\n", frame_grayscale.rows, frame_grayscale.cols, frame_grayscale.channels());


          // Rotate back
//        cv::Point2f center_inv(frame_grayscale.rows / 2., frame_grayscale.rows / 2.);
//        cv::Mat r_inv = cv::getRotationMatrix2D(center_inv, -90, 1.0);
//        cv::warpAffine(frame_grayscale, frame_grayscale, r_inv, cv::Size(frame_grayscale.rows, frame_grayscale.cols));
        cv::rotate(frame_grayscale, frame_grayscale, cv::ROTATE_90_CLOCKWISE);


        //  printf("ROTATED");
        //  cv::cvtColor(frame, frame, CV_YUV2BGR_Y422);
        //    colorbgr_opencv_to_yuv422(frame, img, width, height);
        grayscale_opencv_to_yuv422(frame_grayscale, img, width, height);
        //  printf("After Color converted");


        //outputBGRScale.copyTo(frame(cv::Rect(0, frame.rows - outputBGRScale.rows, outputBGRScale.cols, outputBGRScale.rows)));


        //  // Rotate
        //  cv::Point2f center(divergence_img.cols/2, divergence_img.rows/2);
        //  cv::Mat r = cv::getRotationMatrix2D(center, 90, 1);
        //  cv::warpAffine(divergence_img, divergence_img, r, cv::Size(divergence_img.rows, divergence_img.cols));
        //
        //  // USE GRAYSCALE
        //  divergence_img = divergence_img * 255;
        //  divergence_img.copyTo(frame_grayscale(cv::Rect(0, 0, divergence_img.cols, divergence_img.rows)));
        //  grayscale_opencv_to_yuv422(frame_grayscale, img, width, height);
        //
        //  printf("\nDivergence: ");
        //  for(int i=0; i<thresholded_divergence.cols; i++)
        //      printf(" %d", thresholded_divergence.at<uchar>(0, i));
        //  printf("\n\n\n");
        //    printf("Image: ");
        //    for(int i=0; i<frame.cols; i++)
        //        printf(" %d", frame.at<float>(120, i));
        //    printf("\n\n");


        //  grayscale_opencv_to_yuv422(divergence_img, img, width, height);


        ////  cv::cvtColor(divergence_img, divergence_img, CV_GRAY2BGR);
        //  divergence_img = divergence_img * 255;
        //  printf("Div image:\nheight: %d\nwidth: %d\nchannels: %d\n", divergence_img.rows, divergence_img.cols, divergence_img.channels());
        //  cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_Y422);
        //  printf("frame:\nheight: %d\nwidth: %d\nchannels: %d\ndepth: %d\n", frame.rows, frame.cols, frame.channels(), frame.depth());
        ////  Mat combined_imgs(frame, cv::Rect(0, 0, divergence_img.cols, divergence_img.rows));
        ////  divergence_img.copyTo(combined_imgs);
        //  divergence_img.copyTo(frame(cv::Rect(0, 0, divergence_img.cols, divergence_img.rows)));
        //  printf("frame:\nheight: %d\nwidth: %d\nchannels: %d\ndepth: %d\n", frame.rows, frame.cols, frame.channels(), frame.depth());
        //  colorbgr_opencv_to_yuv422(frame, img, width, height);
        //
        //

        //    Mat divergence_img;
        //    cv::resize(thresholded_divergence, divergence_img, Size(240, 520));
        //    divergence_img.convertTo(divergence_img, CV_8U);
        //    cv::cvtColor(divergence_img, divergence_img, cv::COLOR_GRAY2BGR);
        //
        //    cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_Y422);
        //
        //    divergence_img.copyTo(frame(cv::Rect(0, 0, divergence_img.cols, divergence_img.rows)));
        //
        //    colorbgr_opencv_to_yuv422(frame, img, width, height);



        int lowest_detection_index = find_best_direction_index(thresholded_divergence);


        return lowest_detection_index;
    }
}


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
