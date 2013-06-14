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
#include "sick_laser/motor.h"
#include "sick_laser/capture_image.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <stdio.h>
#include <vector>
#include <fstream>

using namespace std;
//global values
float g_speed = 0.0;
bool g_is_motor_move = false;
float g_motor_deg = 0.0;
char *g_motor_dir;

class SickLaser
{
public:
  SickLaser ();
  ~SickLaser ();
public:
  bool laserIsSaveDataAsPCD (char *p_is_save);
  void laserGetMotorMoveTime (double f_time);
  void laserGetDataCallBack (const sensor_msgs::LaserScan::ConstPtr & laser);
  void laserGetScanDegree (float f_degree);
public:
  CameraCapture m_camera_;
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  vector < double > one_frame_vec_;    //每一帧的vector,此处给出大小，开辟内存空间
  vector < vector < double > > data_vec_;       //原始点云vector,此处给出大小，开辟内存空间
  vector < double > timestamp_vec_;    //save the angle between two laser lines

  bool b_is_save_as_pcd_;
  ros::Time time_start_;
  static float n_count_;
  float first_scan_degree_;
  double motor_move_time_;
  int n_image_count_;
  bool is_captured_;
};

float SickLaser::n_count_ = 0.0;        //the number of frame that the laser has scaned

SickLaser::SickLaser ()
{
  sub_ = nh_.subscribe ("/scan", 1000, &SickLaser::laserGetDataCallBack, this); //create a subscriber

  one_frame_vec_.reserve (361); //save one frame data
  data_vec_.reserve (2000);     //save point cloud 
  timestamp_vec_.reserve (2000);
  b_is_save_as_pcd_ = true;     //whether the point cloud is saved as PCD file. 
  time_start_ = ros::Time::now ();
  first_scan_degree_ = 0.0;
  motor_move_time_ = 0.0;
  n_image_count_ = 1;
  is_captured_ = false;

  m_camera_.camPrintBuildInfo ();
  m_camera_.camConnect ();
}

SickLaser::~SickLaser ()
{
}

bool SickLaser::laserIsSaveDataAsPCD (char *p_is_save)
{
  if (!strcmp (p_is_save, "y"))
  {
    b_is_save_as_pcd_ = true;
  }
  else
    b_is_save_as_pcd_ = false;

  return b_is_save_as_pcd_;
}

void SickLaser::laserGetMotorMoveTime (double f_time)   //get the time stamp
{
  motor_move_time_ = f_time + 0.2;
  //motor_move_time_ = f_time;
}

void SickLaser::laserGetScanDegree (float f_degree)
{
  first_scan_degree_ = f_degree / 180.0 * 3.1415;
}

void SickLaser::laserGetDataCallBack (const sensor_msgs::LaserScan::ConstPtr & laser)
{
  if (g_is_motor_move)
  {
    double duration = ros::Time::now ().toSec () - motor_move_time_;    //here
    if (duration * g_speed < g_motor_deg)
    {
      if (((duration * g_speed - n_image_count_ * 45.0) < 0.5) && (!is_captured_))
      {
        m_camera_.camImageCapture (n_image_count_);
        n_image_count_++;
        is_captured_ = true;
      }
      for (int i = 0; i < 361; ++i)
      {
        one_frame_vec_.push_back (laser->ranges[i]);
      }
      data_vec_.push_back (one_frame_vec_);
      timestamp_vec_.push_back (laser->header.stamp.toSec ());
      one_frame_vec_.clear ();
      n_count_++;
      //ROS_INFO("n_count_ = %f\n",n_count_ );

      if (duration * g_speed - (n_image_count_ - 1) * 45.0 > 10.0)
      {
        is_captured_ = false;
      }
    }
    else
    {
      m_camera_.camStopCapture ();
      float scandegree = duration * g_speed;
      ROS_INFO ("scan %f frame ", n_count_);
      ROS_INFO ("scan degree is: %f", scandegree);
      g_is_motor_move = false;

      float angleResolution = laser->angle_increment;   //  resolution of laser
      //float anglePerFrame = duration.toSec()*g_speed/(n_count_-1)/180*3.14159;// degree between two frames,unit:rad
      //save the timestamp
      ofstream m_of1 ("lasertimestamp.txt");
      ofstream m_of2 ("motortimestamp.txt");
      char buf[50];
      if (m_of1.is_open ())
      {
        for (int i = 0; i < (int) timestamp_vec_.size (); i++)
        {
          bzero (buf, sizeof (buf));
          sprintf (buf, "%lf", timestamp_vec_[i]);
          m_of1 << buf << "\n";
        }
        m_of1.close ();
      }
      else
      {
        ROS_INFO ("open the file lasertimestamp.txt fail");
        return;
      }
      if (m_of2.is_open ())
      {
        sprintf (buf, "%lf", motor_move_time_ - 0.2);
        m_of2 << buf << "\n";
        m_of2.close ();
      }
      else
      {
        ROS_INFO ("open the file motortimestamp.txt fail");
        return;
      }

      vector < double >vecAnglePerFrame;
      vecAnglePerFrame.resize (timestamp_vec_.size ());
      for (int i = 0; i < (int) timestamp_vec_.size (); i++)
      {
        vecAnglePerFrame[i] = (timestamp_vec_[i] - motor_move_time_) * g_speed / 180 * 3.14159;
      }

      int iCloudWidth = 361;    //width of point cloud =  the points number of one frame
      int iCloudHeight = n_count_;      //height of point cloud = the number of frames

      if (b_is_save_as_pcd_)    //save the point cloud as PCD file
      {
        ROS_INFO ("save as a PCD file \n");
        pcl::PointCloud < pcl::PointXYZ > cloudScan;
        cloudScan.width = iCloudWidth;
        cloudScan.height = iCloudHeight;
        cloudScan.is_dense = false;
        cloudScan.points.resize (cloudScan.width * cloudScan.height);

        if (strcmp (g_motor_dir, "0") == 0)
        {
          for (int i = 0; i < iCloudHeight; i++)        //caculate the 3D x,y,z coordinates
            for (int j = 0; j < iCloudWidth; j++)
            {
              cloudScan.points[i * iCloudWidth + j].x =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * cos (first_scan_degree_ - vecAnglePerFrame[i]);
              cloudScan.points[i * iCloudWidth + j].y =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * sin (first_scan_degree_ - vecAnglePerFrame[i]);
              cloudScan.points[i * iCloudWidth + j].z = data_vec_[i][j] * cos (0.0 + j * angleResolution);
            }
        }
        else if (strcmp (g_motor_dir, "1") == 0)
        {
          for (int i = 0; i < iCloudHeight; i++)
            for (int j = 0; j < iCloudWidth; j++)
            {
              cloudScan.points[i * iCloudWidth + j].x =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * cos (first_scan_degree_ + vecAnglePerFrame[i]);
              cloudScan.points[i * iCloudWidth + j].y =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * sin (first_scan_degree_ + vecAnglePerFrame[i]);
              cloudScan.points[i * iCloudWidth + j].z = data_vec_[i][j] * cos (0.0 + j * angleResolution);
            }
        }
        pcl::io::savePCDFileASCII ("laserdata.pcd", cloudScan);

        ROS_INFO ("save end");
      }
      else
      {
        ros::NodeHandle nhCloud;
        ros::Publisher cloud_pub = nhCloud.advertise < sensor_msgs::PointCloud > ("cloud", 50); //create a publisher

        int iCloudWidth = 361;
        int iCloudHeight = n_count_;

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now ();
        cloud.header.frame_id = "sensor_frame";

        cloud.points.resize (iCloudWidth * iCloudHeight);

        cloud.channels.resize (1);
        cloud.channels[0].name = "intensities";

        if (strcmp (g_motor_dir, "0") == 0)
        {
          for (int i = 0; i < iCloudHeight; i++)
            for (int j = 0; j < iCloudWidth; j++)
            {
              cloud.points[i * iCloudWidth + j].x =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * cos (first_scan_degree_ - vecAnglePerFrame[i]);
              cloud.points[i * iCloudWidth + j].y =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * sin (first_scan_degree_ - vecAnglePerFrame[i]);
              cloud.points[i * iCloudWidth + j].z = data_vec_[i][j] * cos (0.0 + j * angleResolution);
            }
        }
        else if (strcmp (g_motor_dir, "1") == 0)
        {
          for (int i = 0; i < iCloudHeight; i++)
            for (int j = 0; j < iCloudWidth; j++)
            {
              cloud.points[i * iCloudWidth + j].x =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * cos (first_scan_degree_ + vecAnglePerFrame[i]);
              cloud.points[i * iCloudWidth + j].y =
                data_vec_[i][j] * sin (0.0 + j * angleResolution) * sin (first_scan_degree_ + vecAnglePerFrame[i]);
              cloud.points[i * iCloudWidth + j].z = data_vec_[i][j] * cos (0.0 + j * angleResolution);
            }
        }
        sleep (3);
        cloud_pub.publish (cloud);
      }
      sleep (1);
      
      return;
    }
  }

}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "getdata");

  int port = atoi (argv[1]);    //'1'->port1, '2'->port2
  g_motor_dir = argv[2];        //motor direction
  int motor_fre = atoi (argv[3]);       //motor frequency
  g_motor_deg = atoi (argv[4]) * 1.0;   //degree to move
  float firstdegree = atoi (argv[5]) * 1.0;
  char *cIsSaveAsPCD = argv[6]; //whethere saved as PCD file

  ROS_INFO ("port is %d", port);
  ROS_INFO ("direction is %s", g_motor_dir);
  ROS_INFO ("frequency is %d", motor_fre);
  ROS_INFO ("degree is %f", g_motor_deg);

  g_speed = motor_fre * STEP_DIS * SUBDIVISION / 180;   //g_speed of motor,unit:degree/second

  Motor myMotor (port, g_motor_dir, motor_fre, g_motor_deg);

  SickLaser laser;
  laser.laserIsSaveDataAsPCD (cIsSaveAsPCD);
  laser.laserGetScanDegree (firstdegree);
  laser.m_camera_.camImageCapture (0);

  double ts = myMotor.motorMove ();
  ROS_INFO ("Motor move at %lf", ts);
  laser.laserGetMotorMoveTime (ts);
  g_is_motor_move = true;

  ros::spin ();

  return 0;
}
