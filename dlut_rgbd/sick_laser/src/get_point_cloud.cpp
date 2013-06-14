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
//=========================================================================
//This software provide a solution to get point cloud and publish it.
//We first get one frame laser data,and then caculate the 3D point cloud 
//acording to the degree that the motor has moved.
//=========================================================================
#include "sick_laser/motor.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <vector>
#include <fstream>

using namespace std;
//global values
float g_speed = 0.0;
bool g_is_move = false;
float g_motor_deg = 0.0;
char *g_motor_dir;

class SickLaser
{
public:
  SickLaser ();
  ~SickLaser ();
public:
  bool laserIsSaveDataAsPCD (char *c_is_save);
  void laserGetMotorMoveTime (double f_time);
  void laserGetDataCallBack (const sensor_msgs::LaserScan::ConstPtr & laser);
  void laserGetScanDegree (float f_degree);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    vector < double >one_frame_vec_;    
    vector < vector < double > >data_vec_;       
    vector < double >timestamp_vec_;    //save the angle between two laser lines

  bool is_save_as_PCD_;
  ros::Time time_start_;
  static float f_count_;
  float first_scan_degree_;
  double motor_move_time_;
};

float SickLaser::f_count_ = 0.0;        //the number of frame that the laser has scaned

SickLaser::SickLaser ()
{
  sub_ = nh_.subscribe ("/scan", 1000, &SickLaser::laserGetDataCallBack, this); //create a subscriber

  one_frame_vec_.reserve (361); //save one frame data
  data_vec_.reserve (800);      //save point cloud 
  timestamp_vec_.reserve (800);
  is_save_as_PCD_ = true;       //whether the point cloud is saved as PCD file. 
  time_start_ = ros::Time::now ();
  first_scan_degree_ = 0.0;
  motor_move_time_ = 0.0;
}

SickLaser::~SickLaser ()
{
}

bool SickLaser::laserIsSaveDataAsPCD (char *c_is_save)
{
  if (!strcmp (c_is_save, "y"))
  {
    is_save_as_PCD_ = true;
  }
  else
    is_save_as_PCD_ = false;

  return is_save_as_PCD_;
}

void SickLaser::laserGetMotorMoveTime (double f_time)   //get the time stamp
{
  motor_move_time_ = f_time + 0.2;
}

void SickLaser::laserGetScanDegree (float f_degree)
{
  first_scan_degree_ = f_degree / 180.0 * 3.1415;
}

void SickLaser::laserGetDataCallBack (const sensor_msgs::LaserScan::ConstPtr & laser)
{
  if (g_is_move)
  {
    double duration = ros::Time::now ().toSec () - motor_move_time_;    //here
    if (duration * g_speed < g_motor_deg)
    {
      for (int i = 0; i < 361; ++i)
      {
        one_frame_vec_.push_back (laser->ranges[i]);
      }
      data_vec_.push_back (one_frame_vec_);
      timestamp_vec_.push_back (laser->header.stamp.toSec ());
      one_frame_vec_.clear ();
      f_count_++;
      ROS_INFO ("f_count_ = %f", f_count_);
    }
    else
    {
      float scandegree = duration * g_speed;
      ROS_INFO ("scan %f frame ", f_count_);
      ROS_INFO ("scan degree is: %f", scandegree);
      g_is_move = false;

      float angleResolution = laser->angle_increment;   //  resolution of laser
      //float anglePerFrame = duration.toSec()*g_speed/(f_count_-1)/180*3.14159;// degree between two frames,unit:rad
      vector < double >vecAnglePerFrame;
      vecAnglePerFrame.resize (timestamp_vec_.size ());
      for (int i = 0; i < (int) timestamp_vec_.size (); i++)
      {
        vecAnglePerFrame[i] = (timestamp_vec_[i] - motor_move_time_) * g_speed / 180 * 3.14159;
      }

      int iCloudWidth = 361;    //width of point cloud =  the points number of one frame
      int iCloudHeight = f_count_;      //height of point cloud = the number of frames

      if (is_save_as_PCD_)      //save the point cloud as PCD file
      {
        ROS_INFO ("save as a PCD file ");
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
        pcl::io::savePCDFileASCII ("laser.pcd", cloudScan);

        ROS_INFO ("save end");
      }
      else
      {
        ros::NodeHandle nhCloud;
        ros::Publisher cloud_pub = nhCloud.advertise < sensor_msgs::PointCloud > ("cloud", 50); //create a publisher

        int iCloudWidth = 361;
        int iCloudHeight = f_count_;

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
  char *is_save_as_pcd = argv[6];       //whethere saved as PCD file

  ROS_INFO ("port is %d\n", port);
  ROS_INFO ("direction is %s\n", g_motor_dir);
  ROS_INFO ("frequency is %d\n", motor_fre);
  ROS_INFO ("degree is %f\n", g_motor_deg);

  g_speed = motor_fre * STEP_DIS * SUBDIVISION / 180;   //g_speed of motor,unit:degree/second

  Motor myMotor (port, g_motor_dir, motor_fre, g_motor_deg);
  SickLaser laser;

  laser.laserIsSaveDataAsPCD (is_save_as_pcd);
  laser.laserGetScanDegree (firstdegree);
  double ts = myMotor.motorMove ();
  laser.laserGetMotorMoveTime (ts);
  g_is_move = true;

  ros::spin ();

  return 0;
}
