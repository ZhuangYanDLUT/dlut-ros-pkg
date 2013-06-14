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
#include <stdio.h>
#include "flycapture/FlyCapture2.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

using namespace FlyCapture2;

void printBuildInfo ()          //print the version information
{
  FC2Version fc2Version;
  Utilities::GetLibraryVersion (&fc2Version);   //Get library version.
  char version[128];

  sprintf (version,
           "FlyCapture2 library version: %d.%d.%d.%d\n",
           fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build);

  ROS_INFO (version);

  char timeStamp[512];
  sprintf (timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__); //get time stamp

  ROS_INFO (timeStamp);
}

void printCameraInfo (CameraInfo * p_cam_info)  //print the information of camera
{
  ROS_INFO ("\n*** CAMERA INFORMATION ***\n"
            "Serial number - %u\n"
            "Camera model - %s\n"
            "Camera vendor - %s\n"
            "Sensor - %s\n"
            "Resolution - %s\n"
            "Firmware version - %s\n"
            "Firmware build time - %s\n\n",
            p_cam_info->serialNumber,
            p_cam_info->modelName,
            p_cam_info->vendorName,
            p_cam_info->sensorInfo,
            p_cam_info->sensorResolution, p_cam_info->firmwareVersion, p_cam_info->firmwareBuildTime);
}

void printError (Error error)
{
  error.PrintErrorTrace ();     //Print a formatted log trace to stderr
}

int runSingleCamera (PGRGuid guid, VideoMode video_mode, FrameRate framerate)   //run one single camera,and get the image data 
{
  Error error;
  Camera cam;

  // Connect to a camera
  error = cam.Connect (&guid);  //Connects the camera object to the camera specified by the GUID.

  if (error != PGRERROR_OK)
  {
    printError (error);
    return -1;
  }
  else
    ROS_INFO ("camera is connected!");

  //set the video mode and framerate of a camera
  error = cam.SetVideoModeAndFrameRate (video_mode, framerate);
  if (error != PGRERROR_OK)
  {
    printError (error);
    
    return -1;
  }
  else
  {
    ROS_INFO ("set the camera OK !");
  }
  // Get the camera information
  CameraInfo cam_info;
  error = cam.GetCameraInfo (&cam_info);        //Retrieves information from the camera such as serial number, model name and other camera information
  if (error != PGRERROR_OK)
  {
    printError (error);
    
    return -1;
  }

  printCameraInfo (&cam_info);

  // Start capturing images
  error = cam.StartCapture ();
  if (error != PGRERROR_OK)
  {
    printError (error);
    
    return -1;
  }

  Image raw_image;              //get the image,raw_image store the image data
  while (cam.RetrieveBuffer (&raw_image) != PGRERROR_OK)
  {
    continue;
  }
  ROS_INFO ("Grabbed one image ");

  //Get the column and row of one image
  unsigned int n_cols = raw_image.GetCols ();
  unsigned int n_rows = raw_image.GetRows ();
  ROS_INFO ("Camera width is : %d", n_cols);
  ROS_INFO ("Camera height is : %d", n_rows);

  //create a publisher named "/pgr_camera/image"
  ros::NodeHandle nh;
  image_transport::ImageTransport it (nh);
  image_transport::Publisher pub = it.advertise ("/pgr_camera/image", 1);
  sensor_msgs::ImagePtr msg;

  // Create a converted image
  Image converted_image;
  IplImage *video_frame = NULL;
  unsigned char *cv_image_data = NULL;

  ros::Rate loop_rate (10);
  unsigned char *p_img_raw = new unsigned char[n_cols * n_rows];
  unsigned char *p_img_color = new unsigned char[n_cols * n_rows * 3];
  while (nh.ok ())
  {
    // Retrieve an image
    error = cam.RetrieveBuffer (&raw_image);
    if (error != PGRERROR_OK)
    {
      printError (error);
      continue;
    }
    // Convert the raw image to the image which you want
    if (video_mode == VIDEOMODE_640x480Y8)
    {
      error = raw_image.Convert (PIXEL_FORMAT_MONO8, &converted_image);
      if (error != PGRERROR_OK)
      {
        printError (error);
        
        return -1;
      }
      //define the opencv picture
      video_frame = cvCreateImage (cvSize (n_cols, n_rows), IPL_DEPTH_8U, 1);
      cv_image_data = (unsigned char *) video_frame->imageData;
      p_img_raw = converted_image.GetData ();
      for (size_t i = 0; i < n_cols * n_rows; ++i)
      {
        cv_image_data[i] = p_img_raw[i];
      }

      msg = sensor_msgs::CvBridge::cvToImgMsg (video_frame, "mono8");   //convert the cv::mat to ros_image message
      msg->header.stamp = ros::Time::now ();

    }
    else
    {
      error = raw_image.Convert (PIXEL_FORMAT_RGB8, &converted_image);  //get the corlor picture
      if (error != PGRERROR_OK)
      {
        printError (error);
        continue;
      }
      //define the opencv picture
      video_frame = cvCreateImage (cvSize (n_cols, n_rows), IPL_DEPTH_8U, 3);
      cv_image_data = (unsigned char *) video_frame->imageData;
      p_img_color = converted_image.GetData ();
      for (size_t i = 0; i < n_cols * n_rows * 3; ++i)
      {
        cv_image_data[i] = p_img_color[i];
      }

      msg = sensor_msgs::CvBridge::cvToImgMsg (video_frame, "bgr8");
      msg->header.stamp = ros::Time::now ();
    }

    pub.publish (msg);          //publish the message

    ros::spin ();
    loop_rate.sleep ();
  }
  if (video_mode == VIDEOMODE_640x480Y8)
  {
    delete[]p_img_raw;
  }
  else
    delete[]p_img_color;

  cvReleaseImage (&video_frame);
  // Stop capturing images
  error = cam.StopCapture ();
  if (error != PGRERROR_OK)
  {
    printError (error);
    return -1;
  }

  // Disconnect the camera
  error = cam.Disconnect ();
  if (error != PGRERROR_OK)
  {
    printError (error);
    
    return -1;
  }

  return 0;
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "image_publisher");

  printBuildInfo ();

  Error error;

  VideoMode m_video_mode;
  FrameRate m_framerate;
  char *p_video_mode = argv[1]; //the parameter is VideoMode that we choise
  char *p_framerate = argv[2];  //the  parameter is framerate that we choise

  ROS_INFO ("VideoMode is : %s", p_video_mode);
  ROS_INFO ("framerate is : %s", p_framerate);

  if (strcmp (p_video_mode, "640x480") == 0)
  {
    m_video_mode = VIDEOMODE_640x480Y8;
  }
  else if (strcmp (p_video_mode, "1280x960") == 0)
  {
    m_video_mode = VIDEOMODE_1280x960Y8;
  }
  if (strcmp (p_framerate, "7.5") == 0)
  {
    m_framerate = FRAMERATE_7_5;
  }
  else if (strcmp (p_framerate, "15") == 0)
  {
    m_framerate = FRAMERATE_15;
  }

  BusManager bus_mgr;

  unsigned int n_camera_num;
  error = bus_mgr.GetNumOfCameras (&n_camera_num);      //Gets the number of cameras attached to the PC
  if (error != PGRERROR_OK)
  {
    printError (error);         //print the error
    
    return -1;
  }

  ROS_INFO ("Number of cameras detected: %u", n_camera_num);

  for (unsigned int i = 0; i < n_camera_num; i++)
  {
    PGRGuid guid;
    error = bus_mgr.GetCameraFromIndex (i, &guid);      //Gets the PGRGuid for a camera on the PC

    if (error != PGRERROR_OK)
    {
      printError (error);
      
      return -1;
    }

    runSingleCamera (guid, m_video_mode, m_framerate);  //run the camera
  }

  return 0;
}
