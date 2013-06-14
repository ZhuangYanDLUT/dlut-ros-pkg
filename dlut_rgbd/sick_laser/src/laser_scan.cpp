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
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define PORT 4001               //The port of Nport
#define IP "192.168.1.213"      //The IP of Nport

int main (int argc, char **argv)
{
  int n_socket;
  struct sockaddr_in socket_addr;
  char buf[2048];

  //create a publisher
  ros::init (argc, argv, "sicklms200");
  ros::NodeHandle nh;
  ros::Publisher sicklms = nh.advertise < sensor_msgs::LaserScan > ("/scan", 1000);

  //create a socket 
  if ((n_socket = socket (AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror ("socket create error!");
    
    return 1;
  }
  else
  {
    ROS_INFO ("socket create successfully\n");
    ROS_INFO ("socket id is: %d\n", n_socket);
  }

  bzero (&socket_addr, sizeof (struct sockaddr_in));
  socket_addr.sin_family = AF_INET;
  socket_addr.sin_port = htons (PORT);
  socket_addr.sin_addr.s_addr = inet_addr (IP);

  //connect to the nport
  if (connect (n_socket, (struct sockaddr *) (&socket_addr), sizeof (struct sockaddr)) < 0)
  {
    perror ("connect error");
    
    return 1;
  }
  else
  {
    ROS_INFO ("connect successfully\n");
    ROS_INFO ("IP is: %s\n", IP);
    ROS_INFO ("PORT is:%d\n", PORT);
  }

  bzero (buf, sizeof (buf));

  unsigned int point_num = 361;
  double laser_frequency = 40;

  //create a LaserScan message,and assign the value
  sensor_msgs::LaserScan sickscan;
  sickscan.header.frame_id = "laser_frame";
  sickscan.angle_min = -1.57;
  sickscan.angle_max = 1.57;
  sickscan.angle_increment = 3.14 / point_num;
  sickscan.time_increment = (1 / laser_frequency) / point_num;
  sickscan.range_min = 0.0;
  sickscan.range_max = 8000.0;

  sickscan.ranges.resize (point_num);
  sickscan.intensities.resize (point_num);
  for (int i = 0; i < (int)point_num; i++)
  {
    sickscan.intensities[i] = 0.0;
  }

  ros::Rate r (40.0);
  bool is_found = false;
  int nrecv_all = 0;
  int head = 0;

  while (ros::ok ())
  {
    int nrecv = recv (n_socket, buf + nrecv_all, sizeof (buf) - nrecv_all, 0);  //receive the data from laser
    if (nrecv == -1)
    {
      perror ("receive error");
      break;
    }
    else
    {
      nrecv_all += nrecv;
      if (nrecv_all < 732)
      {
        continue;
      }
      else
      {
        for (head = 0; head < nrecv_all; head++)        //find the head of one original message,so we can get one intact laser data.
        {
          if (0x02 == (unsigned char) buf[head]
              && 0x80 == (unsigned char) buf[head + 1]
              && 0xD6 == (unsigned char) buf[head + 2] && 0x02 == (unsigned char) buf[head + 3])
          {
            is_found = true;
            break;
          }
        }

        if (is_found && (nrecv_all - head > 732))
        {
          for (int i = head + 7, n = 0; i < head + 7 + 722; i += 2, n++)        //get the intact laser data from the original message,one intact laser data has 361 points.unit(m).
          {
            sickscan.ranges[n] = *((short int *) (buf + i))/1000.0;
          }
          ros::Time scan_time = ros::Time::now ();
          sickscan.header.stamp = scan_time;

          sicklms.publish (sickscan);   //publish the ros_message
          r.sleep ();

          for (int i = head + 732, i_ = 0; i < nrecv_all; i++, i_++)
          {
            buf[i_] = buf[i];
          }
          nrecv_all = nrecv_all - head - 732;
        }
      }
    }
  }

  close (n_socket);

  return 0;
}
