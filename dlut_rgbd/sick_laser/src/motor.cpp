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
#include <ros/ros.h>

void Com::comOpen ()
{
  if (m_comport_ == 1)          //serial port 1
  {
    m_fd_ = open ("/dev/ttyr00", O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == m_fd_)
    {
      perror ("open port 1 FAIL！\n");
    }
    else
    {
      ROS_INFO ("open port 1 SUCCESSFUL");
    }
  }
  else if (m_comport_ == 2)     //serial port 2
  {
    m_fd_ = open ("/dev/ttyr01", O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == m_fd_)
    {
      perror ("open port 2 FAIL！\n");
    }
    else
    {
      ROS_INFO ("open port 2 SUCCESSFUL");
    }
  }

  /*set the port's status */
  if (fcntl (m_fd_, F_SETFL, 0) < 0)
    ROS_INFO ("set the port to Blocked FAIL!");
  else
    ROS_INFO ("set the port to Blocked");
  if (isatty (STDIN_FILENO) == 0)
    ROS_INFO ("It is NOT a terminal equipment");
  else
    ROS_INFO ("It is a tty equipment");
}

void Com::comSet (int n_speed, int n_bits, char c_event, int n_stop)
{
  struct termios newtio, oldtio;
  if (tcgetattr (m_fd_, &oldtio) != 0)
  {
    perror ("save the old parameters FAIL！\n");
  }
  bzero (&newtio, sizeof (newtio));
  /*set character size */
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  /*set data bit */
  switch (n_bits)
  {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }
  /*set parity bit */
  switch (c_event)
  {
    case 'O':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;
    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
  }
  /*set baud rate */
  switch (n_speed)
  {
    case 2400:
      cfsetispeed (&newtio, B2400);
      cfsetospeed (&newtio, B2400);
      break;
    case 4800:
      cfsetispeed (&newtio, B4800);
      cfsetospeed (&newtio, B4800);
      break;
    case 9600:
      cfsetispeed (&newtio, B9600);
      cfsetospeed (&newtio, B9600);
      break;
    case 19200:
      cfsetispeed (&newtio, B19200);
      cfsetospeed (&newtio, B19200);
      break;
    case 38400:
      cfsetispeed (&newtio, B38400);
      cfsetospeed (&newtio, B38400);
      break;
    case 57600:
      cfsetispeed (&newtio, B57600);
      cfsetospeed (&newtio, B57600);
      break;
    case 115200:
      cfsetispeed (&newtio, B115200);
      cfsetospeed (&newtio, B115200);
      break;
    default:
      cfsetispeed (&newtio, B9600);
      cfsetospeed (&newtio, B9600);
      break;
  }
  /*set stop bit */
  if (n_stop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (n_stop == 2)
    newtio.c_cflag |= CSTOPB;

  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush (m_fd_, TCIFLUSH);
  /*activate the new set */
  if ((tcsetattr (m_fd_, TCSANOW, &newtio)) != 0)
  {
    perror ("activate the new set FAIL");
  }
  ROS_INFO ("set port SUCCESSFUL!");
}

void Com::comWrite (char *buf)
{
  int n_write;
  n_write = write (m_fd_, buf, strlen (buf));
  if (n_write < 0)
  {
    ROS_INFO ("write error");
  }
}

void Com::comClose ()
{
  close (m_fd_);
}

int Motor::getPulseNum ()
{
  return (m_degree_ * TRANS_RATIO / STEP_DIS / SUBDIVISION);
}

double Motor::motorMove ()
{
  comOpen ();
  comSet (19200, 8, 'N', 1);    //set the port's parameters
  char buf[30];
  int n_pnum = 0;
  n_pnum = getPulseNum ();
  sprintf(buf, "M%c%06d%06d", m_direction_, m_frequence_, n_pnum);    //get the motor control command with paraters that we set.
  comWrite (buf);
  double time_motor_move = ros::Time::now ().toSec ();

  ROS_INFO ("The command is %s", buf);
  comClose ();

  return time_motor_move;
}
