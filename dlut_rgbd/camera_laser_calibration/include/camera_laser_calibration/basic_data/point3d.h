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
#ifndef POINT3D_H
#define POINT3D_H

class CPoint3d
{
public:
  float x;

  float y;

  float z;

public:
  CPoint3d (void);

  CPoint3d (float x, float y, float z);

  ~CPoint3d (void);

  CPoint3d (const CPoint3d & tmpP);


  CPoint3d & operator += (const CPoint3d & p);

  CPoint3d & operator -= (const CPoint3d & p);

  CPoint3d & operator *= (float s);

  CPoint3d & operator /= (float s);


  friend bool operator > (const CPoint3d & p1, const CPoint3d & p2);

  friend bool operator < (const CPoint3d & p1, const CPoint3d & p2);

  friend bool operator == (const CPoint3d & p1, const CPoint3d & p2);

  friend bool operator != (const CPoint3d & p1, const CPoint3d & p2);

  friend CPoint3d operator + (const CPoint3d & p1, const CPoint3d & p2);

  friend CPoint3d operator - (const CPoint3d & p1, const CPoint3d & p2);

  friend CPoint3d operator * (const CPoint3d & p, float s);

  friend float operator * (const CPoint3d & p1, const CPoint3d & p2);

  friend CPoint3d operator / (const CPoint3d & p, float num);

  friend CPoint3d operator ^ (const CPoint3d & p1, const CPoint3d & p2);


  // The distance between a point and origin
  float cpDist (void) const;

  //Vector dot product
  float cpDot (const CPoint3d & p) const;

  //The square of a vector model
  float cpSquaredNorm () const;

  //The vector units
  CPoint3d & cpNormalize ();
};

#endif
