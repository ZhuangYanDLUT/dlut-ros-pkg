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
#include "camera_laser_calibration/basic_data/point3d.h"
#include <cmath>
//////////////////////////////////////////////////////////////////////////
CPoint3d::CPoint3d (void):x (0), y (0), z (0)
{
}

CPoint3d::CPoint3d (float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

CPoint3d::~CPoint3d (void)
{
}

CPoint3d::CPoint3d (const CPoint3d & tmpP)
{
  this->x = tmpP.x;
  this->y = tmpP.y;
  this->z = tmpP.z;
}

//////////////////////////////////////////////////////////////////////////
CPoint3d & CPoint3d::operator += (const CPoint3d & p)
{
  this->x += p.x;
  this->y += p.y;
  this->z += p.z;
  
  return *this;
}

CPoint3d & CPoint3d::operator -= (const CPoint3d & p)
{
  this->x -= p.x;
  this->y -= p.y;
  this->z -= p.z;
  
  return *this;
}

CPoint3d & CPoint3d::operator *= (float s)
{
  this->x *= s;
  this->y *= s;
  this->z *= s;
  
  return *this;
}

CPoint3d & CPoint3d::operator /= (float s)
{
  this->x /= s;
  this->y /= s;
  this->z /= s;
  
  return *this;
}


CPoint3d operator + (const CPoint3d & p1, const CPoint3d & p2)
{
  CPoint3d po;
  po.x = p1.x + p2.x;
  po.y = p1.y + p2.y;
  po.z = p1.z + p2.z;
  
  return po;
}

CPoint3d operator - (const CPoint3d & p1, const CPoint3d & p2)
{
  CPoint3d po;
  po.x = p1.x - p2.x;
  po.y = p1.y - p2.y;
  po.z = p1.z - p2.z;
  
  return po;
}

CPoint3d operator * (const CPoint3d & p, float s)
{
  CPoint3d po;
  po.x = p.x * s;
  po.y = p.y * s;
  po.z = p.z * s;
  
  return po;
}

float operator * (const CPoint3d & p1, const CPoint3d & p2)
{
  return (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z);
}

CPoint3d operator / (const CPoint3d & p, float num)
{
  if (num != 0)
  {
    CPoint3d po;
    po.x = p.x / num;
    po.y = p.y / num;
    po.z = p.z / num;
    
    return po;
  }
  else
  {
    return CPoint3d (0, 0, 0);
  }
}

CPoint3d operator ^ (const CPoint3d & p1, const CPoint3d & p2)
{
  CPoint3d po (p1.y * p2.z - p1.z * p2.y, p1.z * p2.x - p1.x * p2.z, p1.x * p2.y - p1.y * p2.x);
  
  return po;
}

bool operator < (const CPoint3d & p1, const CPoint3d & p2)
{
  return (p1.z != p2.z) ? (p1.z < p2.z) : (p1.y != p2.y) ? (p1.y < p2.y) : (p1.x < p2.x);
}

bool operator > (const CPoint3d & p1, const CPoint3d & p2)
{
  return (p1.z != p2.z) ? (p1.z > p2.z) : (p1.y != p2.y) ? (p1.y > p2.y) : (p1.x > p2.x);
}

bool operator == (const CPoint3d & p1, const CPoint3d & p2)
{
  if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
    return true;
  else
    return false;
}

bool operator != (const CPoint3d & p1, const CPoint3d & p2)
{
  if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
    return false;
  else
    return true;
}


//////////////////////////////////////////////////////////////////////////
float CPoint3d::cpDist (void) const 
{
  return sqrt (x * x + y * y + z * z);
}


float CPoint3d::cpSquaredNorm () const 
{
  return (this->x * this->x + this->y * this->y + this->z * this->z);
}


float CPoint3d::cpDot (const CPoint3d & p) const 
{
  return (*this) * p;
}


CPoint3d & CPoint3d::cpNormalize ()
{
  float n = float (sqrt (this->x * this->x + this->y * this->y + this->z * this->z));
  if (n > float (0))
  {
    this->x /= n;
    this->y /= n;
    this->z /= n;
  }
  
  return *this;
}
