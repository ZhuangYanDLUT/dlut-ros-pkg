/*********************************************************************
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:Intelligent Robotics Lab, DLUT.
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
#pragma once

#include <vector>
#include <algorithm>
#include <math.h>
#include <string>
#include "PointIndex.h"
#include "../basicData/Point3d.h"

#define MAX_PLANE_NUM 8         // The maximum number of planes
#define MAX_DIST 8              // maximum distance of laser scan unit £ºm
#define DIMEN 6                 //  points of small plane
#define THRESH     0.5          // The lower limit threshold value of field point set's covariance matrices eigenvalue . range:(0.0£¬0.01]. unit :m
#define MAXDISTH1  0.3          // The lower limit threshold value of the distance between points which are in field point set and the barycenter .range(0.0£¬1.0] unit £ºm
const float PI = 3.14159f;
struct Plane;
struct CPointTag;

enum FeatureType                //FeatureTypes
{
  NoFeature,
  Wall,
  Ceiling,
  Floor,
  Noise,
};
// The struct of plane
struct Plane
{
  FeatureType Type;             //plane type
  int VecNum;                   //The number of small plane- templates
  CPoint3d VecSum;              // Sum of normals
  CPoint3d VecAve;              // Plane normals
  CPoint3d MeanVal;             // Plane barycenter
  CPoint3d MeanSum;             // sum of plane barycenter
  float A, B, C, D;             //  parameters of plane
  std::vector < CPointIndex > InnerPoints;    //The index of 3D points in a plane
    Plane ()
  {
    Type = NoFeature;
    VecNum = 0;
    VecSum = CPoint3d (0, 0, 0);
    VecAve = CPoint3d (0, 0, 0);
    MeanSum = CPoint3d (0, 0, 0);
    MeanVal = CPoint3d (0, 0, 0);
    A = B = C = D = 0.0;
  };
};

//The point tag 
struct CPointTag
{
  int pflag;                    //The tag which is used to discriminate between the planes.
  FeatureType Type;             //The tag to discriminate between frame cloud points.
  int g_flag;                   //The tag to discriminate between different objects.
    CPointTag ()
  {
    pflag = -1;
    g_flag = -1;
    Type = NoFeature;
  }
};


class CSceneRecog
{
public:
  int L_width_, L_height_;      //The width and height of a laser point cloud
  std::vector < std::vector < CPoint3d > >laser_data_;        // Raw 3D points
  std::vector < Plane > plane_vector_;        // Vector to save planes.
  std::vector < std::vector < CPointTag > >point_tag_vector_; // One-to-one tag with each points 
  std::vector < std::vector < CPointIndex > >boundary_vec_;   // The rough bounder of each planes.
  std::vector < std::vector < CPointIndex > >convex_vec_;     // Find points in plane convex hull
  float plane_parameters_[4];
public:
  CSceneRecog (void);
   ~CSceneRecog (void);

// **************************************************************************************
  void srProcess (std::vector < std::vector < CPoint3d > >&data);       // Put in the laser data.               
  void srPlaneExtract ();       // Tentative fitting
  void srStructureExtract ();   // consolidate small planes
// ****************************************************************************************************************
  float srPtopDistan (CPoint3d p1, CPoint3d p2);        // distance between points.
  bool srIsAPlane (CPoint3d p[][DIMEN], CPoint3d m);    //
  int srEigenValue (float a[], int n, double eps, int jt);      //Calculate the eigenvalue of a matrix
  CPoint3d srNormalVec (CPoint3d start1, CPoint3d end1, CPoint3d start2, CPoint3d end2);        //
  CPoint3d srVecMuti (CPoint3d start1, CPoint3d end1, CPoint3d start2, CPoint3d end2);  //
  float srAngle (CPoint3d vec1, CPoint3d vec2); //
  bool srCompL1R0bool (const CPointIndex & p1, const CPointIndex & p2, const CPointIndex & p3);
  inline int srCross_Product (const CPointIndex & p1, const CPointIndex & p2, const CPointIndex & p3);
  float srP2Pl (CPoint3d m, const Plane & Plane);
  void srCalColor (int idx, std::vector < float >&rgb);
};
