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
//========================================================================================================
//Here we provide a solution to detect the corners of laser data .
//First,we select the points of chessboard acording the environment.
//Second,we extract the chessboard plane,and caculate the distance between laser and chessboard.
//Third,we detect the corners acording the distance.
//========================================================================================================
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "camera_laser_calibration/scene_recog/scene_recog.h"
#include "camera_laser_calibration/white_black_grid.h"

#define L_MAX 3
#define L_MIN 1
#define Z_MAX 1.9
#define Z_MIN -0.5

int main (int argc, char **argv)
{
  ros::init (argc, argv, "detect_laser_corners");
  char *file_name = argv[1];    //put int the name of pcd file

  pcl::PointCloud < pcl::PointXYZ >::Ptr cloud (new pcl::PointCloud < pcl::PointXYZ >);

  if (pcl::io::loadPCDFile < pcl::PointXYZ > (file_name, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    
    return (-1);
  }
  else
  {
    ROS_INFO ("Read file successfully");
    ROS_INFO ("points number = %d", cloud->width * cloud->height);
  }

  int n_width_raw = cloud->width;       //get the height of row cloud
  int n_height_raw = cloud->height;     //get the width of row cloud

  std::vector < BinIndex > vec_ij;
  BinIndex bin_ij;
  //detect the points of board from a point cloud according to the z value and distance between laser and board.And this is rough.
  for (int j = 0; j < n_height_raw; j++)
    for (int i = 0; i < n_width_raw; i++)
    {
      if ((cloud->points[j * n_width_raw + i].z < Z_MAX) && (cloud->points[j * n_width_raw + i].z > Z_MIN))
      {
        float x = cloud->points[j * n_width_raw + i].x;
        float y = cloud->points[j * n_width_raw + i].y;
        float z = cloud->points[j * n_width_raw + i].z;
        if ((sqrt (x * x + y * y + z * z) > L_MIN) && (sqrt (x * x + y * y + z * z) < L_MAX))
        {
          bin_ij.bin_i = i;
          bin_ij.bin_j = j;
          vec_ij.push_back (bin_ij);
        }
      }
    }
  size_t iMin = vec_ij[0].bin_i;
  size_t iMax = vec_ij[0].bin_i;
  for (int i = 1; i < (int) vec_ij.size (); i++)
  {
    if (vec_ij[i].bin_i > iMax)
    {
      iMax = vec_ij[i].bin_i;
    }
    if (vec_ij[i].bin_i < iMin)
    {
      iMin = vec_ij[i].bin_i;
    }

  }
  size_t jMin = vec_ij[0].bin_j + 5;
  size_t jMax = vec_ij[vec_ij.size () - 1].bin_j - 5;

  ROS_INFO ("iMin = %d\n", iMin);
  ROS_INFO ("iMax = %d\n", iMax);
  ROS_INFO ("jMin = %d\n", jMin);
  ROS_INFO ("jMax = %d\n", jMax);

  int n_width_choosed = iMax - iMin + 1;
  int n_height_choosed = jMax - jMin + 1;

  //find the points of board accurately
  std::vector < CPoint3d > vec_one_frame (n_width_choosed);
  std::vector < std::vector < CPoint3d > >vec_all_frame (n_height_choosed);
  for (unsigned int j = jMin, p = 0; j < jMax + 1; j++, p++)
  {
    for (unsigned int i = iMin, q = 0; i < iMax + 1; i++, q++)
    {
      vec_one_frame[q].x = cloud->points[j * 361 + i].x;
      vec_one_frame[q].y = cloud->points[j * 361 + i].y;
      vec_one_frame[q].z = cloud->points[j * 361 + i].z;
    }
    vec_all_frame[p] = vec_one_frame;
  }
  
  ROS_INFO ("n_width_choosed = %d\n", n_width_choosed);
  ROS_INFO ("n_height_choosed = %d\n", n_height_choosed);

  std::vector < std::vector < CPoint3d > >vec_cloud_raw;
  vec_cloud_raw = vec_all_frame;
  //extract the chessboard plane,and caculate the distance between laser and chessboard
  CSceneRecog scene_recog;
  scene_recog.srProcess (vec_all_frame);
  float plane_parameters[4];
  plane_parameters[0] =  scene_recog.plane_parameters_[0];
  plane_parameters[1] =  scene_recog.plane_parameters_[1];
  plane_parameters[2] =  scene_recog.plane_parameters_[2];
  plane_parameters[3] =  scene_recog.plane_parameters_[3];

  printf ("A = %f,B = %f,C = %f,D = %f\n", plane_parameters[0], plane_parameters[1], plane_parameters[2], plane_parameters[3]);

  //detect the chessboard cornersscene_recog.
  CWhiteBlackGrid whiteblack_grid;
  whiteblack_grid.whgNodeDetect (n_height_choosed, n_width_choosed, plane_parameters, vec_cloud_raw);
  whiteblack_grid.whgNodeModify ();

  return 0;
}
