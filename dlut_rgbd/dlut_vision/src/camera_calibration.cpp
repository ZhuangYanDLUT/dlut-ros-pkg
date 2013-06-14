/*********************************************************************
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:Zhao Cilang,Yan Fei,Zhuang Yan.
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Intelligent Robotics Lab nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cxcore.h>
#include <cv_bridge/CvBridge.h>

int g_boards_n = 20;
const int g_board_dt = 10;
int g_board_w = 9;
int g_board_h = 6;
int g_board_n = g_board_w * g_board_h;
CvSize g_board_sz = cvSize (g_board_w, g_board_h);

    //ALLOCATE STORAGE
CvMat *g_image_points = cvCreateMat (g_boards_n * g_board_n, 2, CV_32FC1);
CvMat *g_object_points = cvCreateMat (g_boards_n * g_board_n, 3, CV_32FC1);
CvMat *g_point_counts = cvCreateMat (g_boards_n, 1, CV_32SC1);

CvMat *g_intrinsic_matrix = cvCreateMat (3, 3, CV_32FC1);
CvMat *g_distortion_coeffs = cvCreateMat (4, 1, CV_32FC1);

CvPoint2D32f *g_corners = new CvPoint2D32f[g_board_n];
int corner_count;
int g_successes = 0;
int g_step, g_frame = 0;

sensor_msgs::CvBridge g_bridge;
IplImage *g_image_raw = NULL;

void imageCallback (const sensor_msgs::ImageConstPtr & msg)
{
  try
  {
    g_image_raw = g_bridge.imgMsgToCv (msg, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException & e)
  {
    ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str ());
  }

  IplImage *gray_image = cvCreateImage (cvGetSize (g_image_raw), 8, 1);
  cvCvtColor (g_image_raw, gray_image, CV_BGR2GRAY);

  if (g_successes < g_boards_n)
  {

    if (g_frame++ % g_board_dt == 0)
    {
      //Find chessboard corners:
      int found = cvFindChessboardCorners (g_image_raw,
                                           g_board_sz,
                                           g_corners,
                                           &corner_count,
                                           CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
      //Get Subpixel accuracy on those corners
      cvFindCornerSubPix (gray_image, g_corners, corner_count,
                          cvSize (11, 11), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30,
                                                                            0.1));

      //Draw corners
      cvDrawChessboardCorners (g_image_raw, g_board_sz, g_corners, corner_count, found);
      cvShowImage ("calibration", g_image_raw);

      // If we got a good board, add it to our data
      if (found == 1)
      {
        g_step = g_successes * g_board_n;
        for (int i = g_step, j = 0; j < g_board_n; ++i, ++j)
        {
          CV_MAT_ELEM (*g_image_points, float, i, 0) = g_corners[j].x;
          CV_MAT_ELEM (*g_image_points, float, i, 1) = g_corners[j].y;
          CV_MAT_ELEM (*g_object_points, float, i, 0) = j / g_board_w;
          CV_MAT_ELEM (*g_object_points, float, i, 1) = j % g_board_w;
          CV_MAT_ELEM (*g_object_points, float, i, 2) = 0.0f;

        }
        CV_MAT_ELEM (*g_point_counts, int, g_successes, 0) = g_board_n;
        g_successes++;
      }
    }

    cvReleaseImage (&gray_image);
    //handle pause/unpause and ESC
    int c = cvWaitKey (15);
    if (c == 'p')
    {
      c = 0;
      while (c != 'p' && c != 27)
      {
        c = cvWaitKey (250);
      }
    }
    if (c == 27)
      return;

  }
  else
  {
    //ALLOCATE MATRICES ACCORDING TO HOW MANY IMAGES WE FOUND CHESSBOARDS ON
    CvMat *object_points2 = cvCreateMat (g_successes * g_board_n, 3, CV_32FC1);
    CvMat *image_points2 = cvCreateMat (g_successes * g_board_n, 2, CV_32FC1);
    CvMat *point_counts2 = cvCreateMat (g_successes, 1, CV_32SC1);
    //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
    for (int i = 0; i < g_successes * g_board_n; ++i)
    {
      CV_MAT_ELEM (*image_points2, float, i, 0) = CV_MAT_ELEM (*g_image_points, float, i, 0);
      CV_MAT_ELEM (*image_points2, float, i, 1) = CV_MAT_ELEM (*g_image_points, float, i, 1);
      CV_MAT_ELEM (*object_points2, float, i, 0) = CV_MAT_ELEM (*g_object_points, float, i, 0);
      CV_MAT_ELEM (*object_points2, float, i, 1) = CV_MAT_ELEM (*g_object_points, float, i, 1);
      CV_MAT_ELEM (*object_points2, float, i, 2) = CV_MAT_ELEM (*g_object_points, float, i, 2);

    }
    for (int i = 0; i < g_successes; ++i)
    {
      CV_MAT_ELEM (*point_counts2, int, i, 0) = CV_MAT_ELEM (*g_point_counts, int, i, 0);
    }
    cvReleaseMat (&g_object_points);
    cvReleaseMat (&g_image_points);
    cvReleaseMat (&g_point_counts);

    // At this point we have all of the chessboard corners we need.  
    // Initialize the intrinsic matrix such that the two focal
    // lengths have a ratio of 1.0
    CV_MAT_ELEM (*g_intrinsic_matrix, float, 0, 0) = 1.0f;
    CV_MAT_ELEM (*g_intrinsic_matrix, float, 1, 1) = 1.0f;
    
    //get the intrinsic parameters
    cvCalibrateCamera2 (object_points2,
                        image_points2,
                        point_counts2, cvGetSize (g_image_raw), g_intrinsic_matrix, g_distortion_coeffs, NULL, NULL, 0);

    cvSave ("Intrinsics.xml", g_intrinsic_matrix);
    cvSave ("Distortion.xml", g_distortion_coeffs);
    cvReleaseImage (&g_image_raw);
    cvReleaseMat (&g_intrinsic_matrix);
    cvReleaseMat (&g_distortion_coeffs);
    cvReleaseMat (&object_points2);
    cvReleaseMat (&image_points2);
    cvReleaseMat (&point_counts2);

    return;
  }
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "camera_calibration");
  ros::NodeHandle nh;
  cvNamedWindow ("calibration");
  cvStartWindowThread ();
  image_transport::ImageTransport it (nh);
  image_transport::Subscriber sub = it.subscribe ("/pgr_camera/image", 1000, imageCallback);    //create a subscriber
  ros::spin ();
  cvDestroyWindow ("calibration");
}
