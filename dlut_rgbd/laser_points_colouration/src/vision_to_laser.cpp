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
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv/highgui.h>
#include <opencv/cv.h>

#include "laser_points_colouration/point3d.h"
#include "laser_points_colouration/point_color.h"

#include <vector>
#include <fstream>
#include <cmath>

#define nWidth 1280
#define nHeight 960

class VisionToLaser
{
public:
  VisionToLaser ();
  ~VisionToLaser ();
public:
  void loadParameters ();       //load the parameters
  void loadLaserPoints (char *_filename);       //load laser data
  void loadVisionImage ();      //load vision data
  void visionToLaser ();        //colored the laser points
  bool saveData ();             //save the colored data
  void getSceneNum (int scene_num_);    //get the scene number
  void getImageMovedDegree ();  //get the degree which image is captured at
private:
    ros::NodeHandle nh_;
private:                       //some matrices and vectors
  CvMat * intrinsic_M_;
  CvMat *rotation_M_;
  CvMat *translate_V_;
  CvMat *target_V_;
  CvMat *far_left_point_;
  CvMat *far_right_point_;

  float left_bound_;            //bounder of points
  float right_bound_;
  float upper_bound_;
  float lower_bound_;
  float front_bound_;
  float back_bound_;

  std::vector < std::vector < CPoint3d > >laser_data_vec_;
  std::vector < std::vector < CPoint3d > >laser_data_raw_vec_;
  std::vector < std::vector < CPointColor > >data_color_vec_;
  IplImage *image[8];

  double image_moved_degree_[8];

  int n_scene_num_;
};

VisionToLaser::VisionToLaser ()
{
  intrinsic_M_ = cvCreateMat (3, 3, CV_32FC1);
  rotation_M_ = cvCreateMat (3, 3, CV_32FC1);
  translate_V_ = cvCreateMat (3, 1, CV_32FC1);
  target_V_ = cvCreateMat (3, 1, CV_32FC1);

  far_left_point_ = cvCreateMat (3, 1, CV_32FC1);
  far_right_point_ = cvCreateMat (3, 1, CV_32FC1);

  left_bound_ = 0.0;
  right_bound_ = 0.0;
  upper_bound_ = 0.0;
  lower_bound_ = 0.0;
  front_bound_ = 0.0;
  back_bound_ = 0.0;

  laser_data_vec_.reserve (2000);
  data_color_vec_.resize (2000);
  laser_data_raw_vec_.reserve (2000);

  for (int i = 0; i < 8; i++)
  {
    image[i] = NULL;
  }

  n_scene_num_ = 0;
}

VisionToLaser::~VisionToLaser ()
{
  cvReleaseMat (&intrinsic_M_);
  cvReleaseMat (&rotation_M_);
  cvReleaseMat (&translate_V_);
  cvReleaseMat (&target_V_);
  cvReleaseMat (&far_left_point_);
  cvReleaseMat (&far_right_point_);
}

void VisionToLaser::loadParameters ()
{
  intrinsic_M_ = (CvMat *) cvLoad ("Intrinsics.xml");   //load intrinsics , translation and rotation matrix
  translate_V_ = (CvMat *) cvLoad ("translation.xml");
  rotation_M_ = (CvMat *) cvLoad ("rotation.xml");

  left_bound_ = 50.0;
  right_bound_ = -50.0;
  upper_bound_ = 50.0;
  lower_bound_ = -50.0;
  front_bound_ = 0.5;
  back_bound_ = 50.0;

}

void VisionToLaser::loadLaserPoints (char *_filename)
{
  pcl::PointCloud < pcl::PointXYZ >::Ptr cloud (new pcl::PointCloud < pcl::PointXYZ >);

  if (pcl::io::loadPCDFile < pcl::PointXYZ > (_filename, *cloud) == -1) //load the pcd file
  {
    PCL_ERROR ("Couldn't read file \n");
    return;
  }
  else
    printf ("points number = %d\n", cloud->width * cloud->height);

  std::vector < CPoint3d > vec_laserdata;
  vec_laserdata.reserve (361);
  CPoint3d p;
  for (unsigned int j = 0; j < cloud->points.size () / 361; j++)
  {
    for (unsigned int i = 0; i < 361; i++)
    {
      p.x = cloud->points[j * 361 + i].x;
      p.y = cloud->points[j * 361 + i].y;
      p.z = cloud->points[j * 361 + i].z;
      vec_laserdata.push_back (p);
    }
    laser_data_raw_vec_.push_back (vec_laserdata);
    vec_laserdata.clear ();
  }
  ROS_INFO ("Read the laser data successfully!");
}

void VisionToLaser::loadVisionImage ()
{
  char _imagename[50];
  for (int _imagenum = 0; _imagenum < 8; _imagenum++)   //load the images
  {
    bzero (_imagename, sizeof (_imagename));
    sprintf (_imagename, "image%d.bmp", _imagenum + 1);
    image[_imagenum] = cvLoadImage (_imagename, CV_LOAD_IMAGE_COLOR);
  }
}

void VisionToLaser::getSceneNum (int scene_num_)
{
  n_scene_num_ = scene_num_;
}

void VisionToLaser::getImageMovedDegree ()
{
  std::ifstream mr_imgtime ("imagetimestamp.txt");      //get the degree from timestamps
  double imgtime[9];
  std::ifstream mr_motortime ("motortimestamp.txt");
  double motortime;
  if (mr_imgtime.is_open ())
  {
    ROS_INFO ("Start read the image timestamp!");
    for (int i = 0; i < 9; i++)
    {
      mr_imgtime >> imgtime[i];
    }
  }
  else
  {
    ROS_INFO ("Read the image timestamp fail");
    
    return;
  }
  mr_imgtime.close ();
  if (mr_motortime.is_open ())
  {
    ROS_INFO ("Start read the motor timestamp!");
    mr_motortime >> motortime;
  }
  else
  {
    ROS_INFO ("Read the motor timestamp fail");
    
    return;
  }
  mr_motortime.close ();
  image_moved_degree_[0] = 0.0; //calculate the degree which images are get at                      
  for (int j = 1; j < 8; j++)
  {
    image_moved_degree_[j] = (imgtime[j + 1] - motortime - 0.2) * 9.0 / 180.0 * 3.1415;
  }
}

void VisionToLaser::visionToLaser ()
{
  size_t laserHeight = laser_data_raw_vec_.size ();
  size_t laserWidth = 361;
  data_color_vec_.resize (laserHeight);
  for (size_t j = 0; j < laserHeight; j++)
    data_color_vec_[j].resize (laserWidth);

  size_t totalPointsNum = laserHeight * laserWidth;
  CvMat *regionPoints = cvCreateMat ((int) totalPointsNum, 3, CV_32FC1);
  CvMat *pointIndex = cvCreateMat ((int) totalPointsNum, 1, CV_32SC1);

  float *regionPtr = NULL;      //regionPoints->data.fl;
  int *pointIndexPtr = NULL;    //pointIndex->data.i;

  float x, y, z;
  size_t counter;
  size_t regionPointsNum;

  unsigned char *img_data;
  int img_step = 0;
  int img_channels = 0;

  int inRangeX;
  int inRangeY;
  size_t reducedPointsNum = 0;

  size_t laser_Index = 0;
  // read in the laser points as cvMat
  std::vector < CPoint3d > vec_data;
  vec_data.reserve (361);
  CPoint3d p;
  for (int scenecount = 0; scenecount < 8; scenecount++)
  {
    for (int j = 0; j < (int) laser_data_raw_vec_.size (); j++)
    {
      for (int i = 0; i < 361; i++)
      {
        float x = laser_data_raw_vec_[j][i].x;
        float y = laser_data_raw_vec_[j][i].y;
        float z = laser_data_raw_vec_[j][i].z;

        p.x = x * cos (image_moved_degree_[scenecount]) - y * sin (image_moved_degree_[scenecount]);
        p.y = x * sin (image_moved_degree_[scenecount]) + y * cos (image_moved_degree_[scenecount]);
        p.z = z;

        vec_data.push_back (p);
      }
      laser_data_vec_.push_back (vec_data);
      vec_data.clear ();
    }

    regionPtr = regionPoints->data.fl;
    pointIndexPtr = pointIndex->data.i;

    counter = 0;
    regionPointsNum = 0;
    while (counter < totalPointsNum)    //judge whether these points are inside the boundary 
    {
      x = laser_data_vec_[counter / laserWidth][counter % laserWidth].x;
      y = laser_data_vec_[counter / laserWidth][counter % laserWidth].y;
      z = laser_data_vec_[counter / laserWidth][counter % laserWidth].z;
      counter++;
      if (y > left_bound_ || y < right_bound_)
      {
        continue;
      }
      else if (z > upper_bound_ || z < lower_bound_)
      {
        continue;
      }
      else if (x > back_bound_ || x < front_bound_)
      {
        continue;
      }
      else
      {
        regionPtr[regionPointsNum * 3 + 0] = x;
        regionPtr[regionPointsNum * 3 + 1] = y;
        regionPtr[regionPointsNum * 3 + 2] = z;
        pointIndexPtr[regionPointsNum] = (int) (counter - 1);
        regionPointsNum++;
      }

    }
    ROS_INFO ("regionPointsNum = %d", regionPointsNum);
    img_data = (unsigned char *) image[scenecount]->imageData;
    img_step = image[scenecount]->widthStep;
    img_channels = image[scenecount]->nChannels;

    reducedPointsNum = 0;
    counter = 0;
    laser_Index = 0;

    while (counter < regionPointsNum)
    {
      target_V_->data.fl[0] = x = regionPtr[counter * 3];
      target_V_->data.fl[1] = y = regionPtr[counter * 3 + 1];
      target_V_->data.fl[2] = z = regionPtr[counter * 3 + 2];
      counter++;
      // project the laser points to iamge coordinates
      cvGEMM (rotation_M_, target_V_, 1.0, translate_V_, 1.0, target_V_, 0);
      cvGEMM (intrinsic_M_, target_V_, 1.0, NULL, 0.0, target_V_, 0);
      // judge if the projected points are in the range 1280*960  
      inRangeX = int (target_V_->data.fl[0] / target_V_->data.fl[2] + 0.5);
      if (inRangeX < 0 || inRangeX >= nWidth)
      {
        continue;
      }
      inRangeY = int (target_V_->data.fl[1] / target_V_->data.fl[2] + 0.5);
      if (inRangeY < 0 || inRangeY >= nHeight)
      {
        continue;
      }

      laser_Index = pointIndexPtr[counter - 1];
      /*if (data_color_vec_[laser_Index/laserWidth][laser_Index%laserWidth].bIsColored == false)
         {
         data_color_vec_[laser_Index/laserWidth][laser_Index%laserWidth].r =
         img_data[inRangeY*img_step+inRangeX*img_channels+2]/255.0;
         data_color_vec_[laser_Index/laserWidth][laser_Index%laserWidth].g =
         img_data[inRangeY*img_step+inRangeX*img_channels+1]/255.0;
         data_color_vec_[laser_Index/laserWidth][laser_Index%laserWidth].b =
         img_data[inRangeY*img_step+inRangeX*img_channels]/255.0;
         data_color_vec_[laser_Index/laserWidth][laser_Index%laserWidth].bIsColored = true;
         reducedPointsNum++;
         } */
      //colored the laser points with image
      data_color_vec_[laser_Index / laserWidth][laser_Index % laserWidth].r =
        img_data[inRangeY * img_step + inRangeX * img_channels + 2] / 255.0;
      data_color_vec_[laser_Index / laserWidth][laser_Index % laserWidth].g =
        img_data[inRangeY * img_step + inRangeX * img_channels + 1] / 255.0;
      data_color_vec_[laser_Index / laserWidth][laser_Index % laserWidth].b =
        img_data[inRangeY * img_step + inRangeX * img_channels] / 255.0;
      data_color_vec_[laser_Index / laserWidth][laser_Index % laserWidth].bIsColored = true;
      reducedPointsNum++;
    }
    ROS_INFO ("This is the %d scenes", scenecount);
    laser_data_vec_.clear ();
  }

  cvReleaseMat (&regionPoints);
  cvReleaseMat (&pointIndex);
}

bool VisionToLaser::saveData () //save colored points
{
  std::ofstream m_w ("colordata.txt");
  if (!m_w.is_open ())
  {
    ROS_INFO ("Open the file colordata.txt fail\n");
    
    return false;
  }
  else
  {
    ROS_INFO ("Open the file ok,and write the colored points to this file!");
    for (int j = 0; j < (int) laser_data_raw_vec_.size (); j++)
      for (int i = 0; i < 361; i++)
      {
        m_w << laser_data_raw_vec_[j][i].x << "  " << laser_data_raw_vec_[j][i].y << "  " << laser_data_raw_vec_[j][i].
          z << "  " << data_color_vec_[j][i].r << "  " << data_color_vec_[j][i].g << "  " << data_color_vec_[j][i].
          b << "  " << data_color_vec_[j][i].bIsColored << "\n";
      }
    m_w.close ();
    
    return true;
  }
}

int main (int argc, char **argv)
{
  char *pointcloud_name = argv[1];      //please put in the pcd file name

  ros::init (argc, argv, "colouration");

  VisionToLaser m_vtl;

  m_vtl.loadParameters ();
  m_vtl.loadLaserPoints (pointcloud_name);
  m_vtl.getImageMovedDegree ();

  m_vtl.loadVisionImage ();
  m_vtl.visionToLaser ();

  bool isSaved = m_vtl.saveData ();
  if (isSaved)
  {
    ROS_INFO ("Save the colored data successfully!");
  }
  else
  {
    ROS_INFO ("Save the colored data fail!");
    
    return -1;
  }

  return 0;
}
