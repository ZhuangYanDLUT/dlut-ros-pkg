/*********************************************************************
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:Zhao Cilang,Yan Fei,Zhuang Yan
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
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <vector>
#include <fstream>

#define nWidth  1280
#define nHeight 960
#define board_w 3
#define board_h 4
#define FOREGROUND 255
#define BACKGROUND 0

using namespace std;

bool integralImageThresholding (IplImage * src, IplImage * img_threshold, double t);

bool integralImageThresholding (IplImage * src, IplImage * img_threshold, double t)
{
  //safety test of the input image
  if (src == NULL || src->width != img_threshold->width || src->height != img_threshold->height)
    return false;

  //initialization of the parameters
  int w = src->width;
  int h = src->height;
  int s = w / 8;                /* The sliding window */
  int sum, num;
  int x1, y1, x2, y2;
  vector < vector < int > >integral;     /*Store the integral image */

  //Initialize the integral image
  integral.resize (h);
  for (int i = 0; i < h; i++)
    integral[i].resize (w);

  //calculate the integral image
  for (int i = 0; i < h; i++)
  {
    sum = 0;
    uchar *ptr = (uchar *) (src->imageData + src->widthStep * i);
    for (int j = 0; j < w; j++)
    {
      sum += ptr[j];
      if (0 == i)
        integral[i][j] = sum;
      else
        integral[i][j] = integral[i - 1][j] + sum;
    }
  }

  //Perform the adaptive threshold
  for (int i = 0; i < h; i++)
  {
    uchar *ptr = (uchar *) (src->imageData + src->widthStep * i);
    uchar *out = (uchar *) (img_threshold->imageData + img_threshold->widthStep * i);
    for (int j = 0; j < w; j++)
    {
      x1 = j - s / 2;
      x2 = j + s / 2;
      y1 = i - s / 2;
      y2 = i + s / 2;

      //Image border inspection
      if (x1 < 0)
        x1 = 0;
      if (x2 >= w)
        x2 = w - 1;
      if (y1 < 0)
        y1 = 0;
      if (y2 >= h)
        y2 = h - 1;

      num = (x2 - x1) * (y2 - y1);

      // calculate  the 'sum' according to different boundary conditions
      if (0 == x1 && y1 != 0)
        sum = integral[y2][x2] - integral[y1 - 1][x2];
      else if (0 == y1 && x1 != 0)
        sum = integral[y2][x2] - integral[y2][x1 - 1];
      else if (0 == x1 && 0 == y1)
        sum = integral[y2][x2];
      else
        sum = integral[y2][x2] - integral[y2][x1 - 1] - integral[y1 - 1][x2] + integral[y1 - 1][x1 - 1];

      //compare the adaptive threshold values 
      if ((int) (ptr[j]) * num <= sum * (1 - t))
        out[j] = FOREGROUND;
      else
        out[j] = BACKGROUND;
    }
  }
  return true;
}


int main (int argc, char **argv)
{
  char *filename = argv[1];
  int r_t = atoi (argv[2]);
  int bg_t = atoi (argv[3]);
  int bg_diff = atoi (argv[4]);

  IplImage *image = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);        //load the image
  IplImage *raw_image = cvCreateImage (cvSize (nWidth, nHeight), 8, 3);
  cvCopy (image, raw_image, NULL);

  int **arr_image = new int *[nHeight];
  for (int i = 0; i < nHeight; i++)
  {
    arr_image[i] = new int[nWidth];
  }

  int x, y;
  int r, g, b;
  vector < CvPoint > red_points_vec;
  vector < CvPoint > detect_points_vec;
  vector < vector < CvPoint > >result_points_vec;
  CvPoint point;

  for (y = 0; y < nHeight; y++)
  {
    for (x = 0; x < nWidth; x++)
    {
      r = ((unsigned char *) (image->imageData + image->widthStep * y))[x * 3 + 2];
      g = ((unsigned char *) (image->imageData + image->widthStep * y))[x * 3 + 1];
      b = ((unsigned char *) (image->imageData + image->widthStep * y))[x * 3];

      if ((r > r_t) && (g < bg_t) && (b < bg_t) && (abs (g - b) < bg_diff))     //find the red points
      {
        arr_image[y][x] = 1;
        point.x = x;
        point.y = y;
        red_points_vec.push_back (point);
      }
      else
        arr_image[y][x] = 0;
    }
  }
  detect_points_vec.reserve (red_points_vec.size ());   //save the points we want to detect
  result_points_vec.reserve (red_points_vec.size ());   //save the result 
  for (int i = 0; i < (int) red_points_vec.size (); i++)        //clustering to find the red  border
  {
    if (arr_image[red_points_vec[i].y][red_points_vec[i].x] == 1)
    {
      arr_image[red_points_vec[i].y][red_points_vec[i].x] = 0;
      detect_points_vec.push_back (red_points_vec[i]);
      vector < CvPoint >::iterator ptr = detect_points_vec.begin ();
      while (ptr != detect_points_vec.end ())
      {
        if (arr_image[ptr->y - 1][ptr->x] == 1)
        {
          point.x = ptr->x;
          point.y = ptr->y - 1;
          detect_points_vec.push_back (point);
          arr_image[ptr->y - 1][ptr->x] = 0;
        }
        if (arr_image[ptr->y][ptr->x + 1] == 1)
        {
          point.x = ptr->x + 1;
          point.y = ptr->y;
          detect_points_vec.push_back (point);
          arr_image[ptr->y][ptr->x + 1] = 0;
        }
        if (arr_image[ptr->y + 1][ptr->x] == 1)
        {
          point.x = ptr->x;
          point.y = ptr->y + 1;
          detect_points_vec.push_back (point);
          arr_image[ptr->y + 1][ptr->x] = 0;
        }
        if (arr_image[ptr->y][ptr->x - 1] == 1)
        {
          point.x = ptr->x - 1;
          point.y = ptr->y;
          detect_points_vec.push_back (point);
          arr_image[ptr->y][ptr->x - 1] = 0;
        }
        ptr++;
      }
      result_points_vec.push_back (detect_points_vec);  //save the clustering result 
      detect_points_vec.clear ();
      ROS_INFO ("Find a class here");
    }
    else
    {
      continue;
    }
  }

  for (int i = 0; i < nHeight; i++)
  {
    delete[]arr_image[i];
  }
  delete[]arr_image;
  ROS_INFO ("There are all %d classes", result_points_vec.size ());

  int maxSize = (int) result_points_vec[0].size ();
  int k = 0;
  for (int i = 1; i < (int) result_points_vec.size (); i++)     //find the biggest class ,which is the red border
  {
    if ((int) result_points_vec[i].size () > maxSize)
    {
      maxSize = (int) result_points_vec[i].size ();
      k = i;
    }

  }
  ROS_INFO ("The biggst class has %d points", maxSize);

  int min_x, min_y, max_x, max_y;
  min_x = result_points_vec[k][0].x;
  min_y = result_points_vec[k][0].y;
  max_x = result_points_vec[k][0].x;
  max_y = result_points_vec[k][0].y;

  for (int i = 2; i < (int) result_points_vec[k].size (); i++)  //find the minimum and maximum of x and y 
  {
    if (result_points_vec[k][i].x > max_x)
    {
      max_x = result_points_vec[k][i].x;
    }
    if (result_points_vec[k][i].y > max_y)
    {
      max_y = result_points_vec[k][i].y;
    }
    if (result_points_vec[k][i].x < min_x)
    {
      min_x = result_points_vec[k][i].x;
    }
    if (result_points_vec[k][i].y < min_y)
    {
      min_y = result_points_vec[k][i].y;
    }
  }
  CvPoint min_point, max_point;
  min_point.x = min_x;
  min_point.y = min_y;
  max_point.x = max_x;
  max_point.y = max_y;
  cvRectangle (image, min_point, max_point, CV_RGB (0, 200, 0), 8, 8, 0);       //draw the border

  //cut the chess board from the whole image 
  CvScalar s;
  IplImage *chessboard_image =
    cvCreateImage (cvSize (max_point.x - min_point.x + 1, max_point.y - min_point.y + 1), 8, 3);
  for (int i = min_point.y; i < max_point.y; i++)
  {
    for (int j = min_point.x; j < max_point.x; j++)
    {
      s.val[2] = ((unsigned char *) (raw_image->imageData + raw_image->widthStep * i))[j * 3 + 2];
      s.val[1] = ((unsigned char *) (raw_image->imageData + raw_image->widthStep * i))[j * 3 + 1];
      s.val[0] = ((unsigned char *) (raw_image->imageData + raw_image->widthStep * i))[j * 3];
      cvSet2D (chessboard_image, i - min_point.y, j - min_point.x, s);
    }
  }

  IplImage *chessboard_grayimage_ = cvCreateImage (cvGetSize (chessboard_image), 8, 1);
  cvCvtColor (chessboard_image, chessboard_grayimage_, CV_RGB2GRAY);
  IplImage *chessboard_grayimage = cvCreateImage (cvGetSize (chessboard_image), 8, 1);
  bool bIsInteg = integralImageThresholding (chessboard_grayimage_, chessboard_grayimage, 0.15);        // do integral for image and thresholding 
  if (bIsInteg)
  {
    ROS_INFO ("GOOD");
  }
  else
  {
    ROS_INFO ("Fail to thresholding");
    return -1;
  }

  CvPoint2D32f *corners = new CvPoint2D32f[board_w * board_h];
  int corner_count;
  CvSize board_sz = cvSize (board_w, board_h);
  //find the corners of chessboard
  int found = cvFindChessboardCorners (chessboard_grayimage,
                                       board_sz,
                                       corners,
                                       &corner_count,
                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  ROS_INFO ("CORNER_COUMT = %d", corner_count);
  if (found == 1)
  {
    ROS_INFO ("FIND CORNERS OK");
  }
  else
    ROS_INFO ("FAIL TO FIND CORNERS");

  cvFindCornerSubPix (chessboard_grayimage, corners, corner_count, cvSize (11, 11), cvSize (-1, -1),
                      cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  CvPoint corner_point[corner_count];
  CvPoint corner_point_raw[corner_count];
  //Save the corners
  ofstream my_w ("visionpts.txt");
  if (!my_w.is_open ())
  {
    ROS_INFO ("open the file visionpts.txt fail\n");
    return -1;
  }
  else
    ROS_INFO ("open the file ok\n");

  for (int i = 0; i < corner_count; i++)
  {
    corner_point[i].x = corners[i].x;
    corner_point[i].y = corners[i].y;
    corner_point_raw[i].x = corners[i].x + min_point.x;
    corner_point_raw[i].y = corners[i].y + min_point.y;
    my_w << corner_point_raw[i].x * 1.0 << "  " << corner_point_raw[i].y * 1.0 << "\n";

  }
  my_w.close ();
  //draw the corners
  for (int i = 0; i < corner_count; i++)
  {
    cvCircle (chessboard_grayimage, corner_point[i], 5, CV_RGB (200, 0, 0));
    cvCircle (raw_image, corner_point_raw[i], 5, CV_RGB (200, 0, 0));

  }
  delete[]corners;

  cvNamedWindow ("detect_egde", 0);
  cvNamedWindow ("detect_corners", 0);
  cvNamedWindow ("raw_image", 0);
  cvShowImage ("detect_egde", image);
  cvShowImage ("detect_corners", chessboard_grayimage);
  cvShowImage ("raw_image", raw_image);
  cvWaitKey (0);
  cvDestroyWindow ("detect_egde");
  cvDestroyWindow ("detect_corners");
  cvDestroyWindow ("raw_image");
  cvReleaseImage (&image);
  cvReleaseImage (&chessboard_grayimage);
  cvReleaseImage (&raw_image);

  return 0;
}
