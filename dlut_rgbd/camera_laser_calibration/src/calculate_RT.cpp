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
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <fstream>
#include <vector>

using namespace std;

int main (int argc, char **argv)
{
  int n_first_num = atoi (argv[1]);     //the number of first file
  int n_file_num = atoi (argv[2]);      //the number of files we read in
  ifstream inf_vision;
  ifstream inf_laser;
  std::vector < float >vision_pts_vec;
  std::vector < float >laser_pts_vec;

  //read in the vision data
  for (int i = n_first_num; i < n_file_num + n_first_num; i++)
  {
    char filename[100];
    sprintf (filename, "visionpts%d.txt", i);
    inf_vision.open (filename);
    if (inf_vision.fail ())
    {
      ROS_INFO ("can not open file");
      return -1;
    }
    float f;
    while (inf_vision >> f)
    {
      vision_pts_vec.push_back (f);
      inf_vision >> f;
      vision_pts_vec.push_back (f);
    }
    inf_vision.clear ();
    inf_vision.close ();
  }

  //read in the laser data
  for (int i = n_first_num; i < n_file_num + n_first_num; i++)
  {
    char filename[100];
    sprintf (filename, "laserpts%d.txt", i);
    inf_laser.open (filename);
    if (inf_laser.fail ())
    {
      ROS_INFO ("can not open file");
      return -1;
    }
    float f;
    while (inf_laser >> f)
    {
      laser_pts_vec.push_back (f);
      for (int j = 0; j < 2; j++)
      {
        inf_laser >> f;
        laser_pts_vec.push_back (f);
      }
    }
    inf_laser.clear ();
    inf_laser.close ();
  }

  int pts_num = laser_pts_vec.size () / 3;
  if (pts_num != (int) vision_pts_vec.size () / 2)
  {
    ROS_INFO ("Numbers not match ");
    
    return -1;
  }
  //define some CvMat
  CvMat *p_image_points = cvCreateMat (pts_num, 2, CV_32FC1);
  CvMat *p_object_points = cvCreateMat (pts_num, 3, CV_32FC1);
  CvMat *intrinsic_M = cvCreateMat (3, 3, CV_32FC1);
  CvMat *rotation_V = cvCreateMat (3, 1, CV_32FC1);
  CvMat *distortion_V = cvCreateMat (4, 1, CV_32FC1);
  CvMat *rotation_M = cvCreateMat (3, 3, CV_32FC1);
  CvMat *translate_V = cvCreateMat (3, 1, CV_32FC1);

  //read in the intrinsic parameters
  intrinsic_M = (CvMat *) cvLoad ("Intrinsics.xml");
  distortion_V = (CvMat *) cvLoad ("Distortion.xml");
  cvSetZero (distortion_V);

  float *ptrVision = p_image_points->data.fl;
  float *ptrLaser = p_object_points->data.fl;
  //save the vision corners as the order of laser corners
  float tempPoints[100][2];
  const int numperimg = 12;
  for (int imgcnt = 0; imgcnt < 48; imgcnt += numperimg)
  {
    for (int i = 0; i < numperimg; i += 4)
    {
      for (int j = 0; j < 4; j++)
      {
        tempPoints[imgcnt + i + j][0] = vision_pts_vec[(imgcnt + i / 4 + j * 3) * 2];
        tempPoints[imgcnt + i + j][1] = vision_pts_vec[(imgcnt + i / 4 + j * 3) * 2 + 1];
      }
    }
  }

  for (int imgP = 0; imgP < pts_num; imgP++)
  {
    ptrVision[2 * imgP] = tempPoints[imgP][0];
    ptrVision[2 * imgP + 1] = tempPoints[imgP][1];
  }

  for (int selP = 0; selP < pts_num; selP++)
  {
    ptrLaser[3 * selP] = laser_pts_vec[3 * selP];
    ptrLaser[3 * selP + 1] = laser_pts_vec[3 * selP + 1];
    ptrLaser[3 * selP + 2] = laser_pts_vec[3 * selP + 2];
  }

  //calculate the R,t use the OpenCV function
  cvFindExtrinsicCameraParams2 (p_object_points, p_image_points, intrinsic_M, distortion_V, rotation_V, translate_V);
  cvRodrigues2 (rotation_V, rotation_M, NULL);
  cvSave ("rotation.xml", rotation_M);  //save the parameters
  cvSave ("translation.xml", translate_V);
  cvReleaseMat (&p_image_points);       //release these matrices
  cvReleaseMat (&p_object_points);
  cvReleaseMat (&intrinsic_M);
  cvReleaseMat (&rotation_V);
  cvReleaseMat (&rotation_M);
  cvReleaseMat (&translate_V);

  return 0;
}
