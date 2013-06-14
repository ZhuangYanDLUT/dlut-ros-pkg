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
#include "sick_laser/capture_image.h"

using namespace FlyCapture2;

CameraCapture::CameraCapture ()
{
  m_videomode_ = VIDEOMODE_1280x960Y8;
  m_framerate_ = FRAMERATE_15;
  m_wfile_.open ("imagetimestamp.txt");
  if (m_wfile_.is_open ())
  {
    ROS_INFO ("open the file ok!");
  }
  else
  {
    ROS_INFO ("open the file fail!");
    
    return;
  }
}

CameraCapture::~CameraCapture ()
{
  //m_wfile_.close();
}

void CameraCapture::camPrintBuildInfo ()
{
  FC2Version fc2Version;        // FC2Version is a struct of  current version of the library
  Utilities::GetLibraryVersion (&fc2Version);   //Get library version.
  char version[128];

  sprintf (version,
           "FlyCapture2 library version: %d.%d.%d.%d",
           fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build);

  ROS_INFO ("%s", version);

  char timeStamp[512];
  sprintf (timeStamp, "Application build date: %s %s", __DATE__, __TIME__);     //get time stamp

  ROS_INFO ("%s", timeStamp);
}

void CameraCapture::camPrintCameraInfo (CameraInfo * cam_info)
{
  ROS_INFO ("\n*** CAMERA INFORMATION ***\n"
            "Serial number - %u\n"
            "Camera model - %s\n"
            "Camera vendor - %s\n"
            "Sensor - %s\n"
            "Resolution - %s\n"
            "Firmware version - %s\n"
            "Firmware build time - %s\n\n",
            cam_info->serialNumber,
            cam_info->modelName,
            cam_info->vendorName,
            cam_info->sensorInfo, cam_info->sensorResolution, cam_info->firmwareVersion, cam_info->firmwareBuildTime);
}

void CameraCapture::camPrintError (Error error)
{
  error.PrintErrorTrace ();
}

void CameraCapture::camConnect ()
{
  Error error;
  error = m_bus_mgr_.GetCameraFromIndex (0, &m_guid_);  //Gets the PGRGuid for a camera on the PC

  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

  error = m_cam_.Connect (&m_guid_);
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }
  else
    ROS_INFO ("camera is connected!");

  //set the video mode and framerate of a camera
  error = m_cam_.SetVideoModeAndFrameRate (m_videomode_, m_framerate_);
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }
  else
  {
    ROS_INFO ("set the camera OK !");
  }

  // Get the camera information
  CameraInfo camInfo;           //CameraInfo is a struct of a camera's information 
  error = m_cam_.GetCameraInfo (&camInfo);      //Retrieves information from the camera such as serial number, model name and other camera information
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

  camPrintCameraInfo (&camInfo);

  error = m_cam_.StartCapture ();
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

}

void CameraCapture::camImageCapture (int img_num)
{
  Error error;
  Image rawImage;
  while (m_cam_.RetrieveBuffer (&rawImage) != PGRERROR_OK)
  {
    //get the image,rawImage store the image data
    continue;
  }

  //save the timestamp
  double time_now = ros::Time::now ().toSec ();
  char buf[50];
  bzero (buf, sizeof (buf));
  sprintf (buf, "%lf", time_now);
  m_wfile_ << buf << "\n";

  // Create a converted image
  Image convertedImage;
  error = rawImage.Convert (PIXEL_FORMAT_BGR, &convertedImage);
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

  // Create a unique filename
  char filename[512];
  sprintf (filename, "image%d.bmp", img_num);

  // Save the image. If a file format is not passed in, then the file
  // extension is parsed to attempt to determine the file format.
  error = convertedImage.Save (filename);
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }
  else
  {
    ROS_INFO ("Have saved %d pictures", img_num + 1);
  }
}

void CameraCapture::camStopCapture ()
{
  // Stop capturing images
  Error error;
  error = m_cam_.StopCapture ();
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

  // Disconnect the camera
  error = m_cam_.Disconnect ();
  if (error != PGRERROR_OK)
  {
    camPrintError (error);
    
    return;
  }

  m_wfile_.close ();
}
