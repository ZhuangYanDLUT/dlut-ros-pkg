/*********************************************************************
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:Zhao Cilang
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
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#define TRANS_RATIO 180         //transmission ratio
#define STEP_DIS 1.8            //step angle
#define SUBDIVISION  0.25       //subdividing number

class Com
{
public:
  Com (int _comport)
  {
    m_fd_ = 0;
    m_comport_ = _comport;
  }                             //initialize the port,m_comport_ is the port number.
   ~Com ()
  {
  }

public:
  void comOpen ();              //open the port
  void comSet (int n_speed, int n_bits, char c_event, int n_stop);      //set the port
  void comWrite (char *buf);    //write to the port
  void comClose ();             //close the port
private:
  int m_fd_;
  int m_comport_;
};

class Motor:public Com
{
public:
  Motor (int _port, char *_direction, int _frequence, int _degree):Com (_port)
  {
    if (strcmp (_direction, "0") == 0)
    {
      m_direction_ = '0';
    }
    if (strcmp (_direction, "1") == 0)
    {
      m_direction_ = '1';
    }
    m_degree_ = _degree;
    m_frequence_ = _frequence;
  }

  ~Motor ()
  {
  }
public:
  int getPulseNum ();           //caculate the pulse number
  double motorMove ();          //get the motor to move
private:
  char m_direction_;
  int m_degree_;
  int m_frequence_;
};
