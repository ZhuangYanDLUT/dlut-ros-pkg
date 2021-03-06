/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Fei Yan
 *
 *  All rights reserved.
 *
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

#include "dlut_place_recognition/bearing_angle_image.h"

BearingAngleImage::BearingAngleImage ()
{
}

BearingAngleImage::~BearingAngleImage ()
{
  cvReleaseImage (&BA_image);
  cvReleaseImage (&channels_image);
}


// Calculate the angle between the laser beam and the segment joining two consecutive measurement points
double BearingAngleImage::getAngle (const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
{
  double a, b, c;
  double theta;
  a = sqrt (point1.x * point1.x + point1.y * point1.y + point1.z * point1.z);
  b = sqrt ((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) +
            (point1.z - point2.z) * (point1.z - point2.z));
  c = sqrt (point2.x * point2.x + point2.y * point2.y + point2.z * point2.z);

  if (a != 0 && b != 0)
  {
    theta = acos ((a * a + b * b - c * c) / (2 * a * b)) * 180 / M_PI;
  }
  else
  {
    theta = 0;
  }

  return theta;
}

// Based on the theta, calculate the gray value of every pixel point
double BearingAngleImage::getGray (double theta)
{
  double gray;
  gray = theta / 180 * 255;
  return gray;
}

// Transform 3D point cloud into a 2D improved bearing-angle(BA) image
IplImage* BearingAngleImage::generateBAImage (std::vector< std::vector<pcl::PointXYZ> > &points, int width, int height)
{
  BA_image = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 1);
  cvSetZero (BA_image);

  int gray;
  double theta;
  pcl::PointXYZ current_point, next_point;

  // Primary transformation process
  for (int i = 1; i < (int) points.size (); ++i)
  {
    for (int j = 0; j < (int) points[i].size () - 1; ++j)
    {
      current_point.x = points[i][j].x;
      current_point.y = points[i][j].y;
      current_point.z = points[i][j].z;
      next_point.x = points[i - 1][j + 1].x;
      next_point.y = points[i - 1][j + 1].y;
      next_point.z = points[i - 1][j + 1].z;

      theta = getAngle (current_point, next_point);
      gray = getGray (theta);
      /* Set the gray value for every pixel point */
      cvSetReal2D (BA_image, height - i - 1, width - j - 1, gray);
    }
  }

  return BA_image;
}

IplImage* BearingAngleImage::getChannelsImage (IplImage* ipl_image)
{
  /* Create a three channels image */
  channels_image = cvCreateImage (cvSize (ipl_image->width, ipl_image->height), IPL_DEPTH_8U, 3);
  cvCvtColor (ipl_image, channels_image, CV_GRAY2BGR);
  return channels_image;
}

QImage BearingAngleImage::cvIplImage2QImage (IplImage* ipl_image)
{
  int width = ipl_image->width;
  int height = ipl_image->height;

  if (ipl_image->depth == IPL_DEPTH_8U && ipl_image->nChannels == 3)
  {
    const uchar *qImageBuffer = (const uchar*) ipl_image->imageData;
    QImage img (qImageBuffer, width, height, QImage::Format_RGB888);
    return img.rgbSwapped ();
  }
  else if (ipl_image->depth == IPL_DEPTH_8U && ipl_image->nChannels == 1)
  {
    const uchar *qImageBuffer = (const uchar*) ipl_image->imageData;
    QImage img (qImageBuffer, width, height, QImage::Format_Indexed8);

    QVector < QRgb > colorTable;
    for (int i = 0; i < 256; i++)
    {
      colorTable.push_back (qRgb (i, i, i));
    }
    img.setColorTable (colorTable);
    return img;
  }
  else
  {
    PCL_ERROR ("Error\n Image cannot be converted.\n");
    return QImage ();
  }
}