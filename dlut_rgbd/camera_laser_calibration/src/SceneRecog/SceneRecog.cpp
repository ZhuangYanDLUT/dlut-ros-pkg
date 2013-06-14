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
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <ctype.h>
#include <ctime>
#include "SceneRecog.h"

#include <ros/ros.h>

CSceneRecog *p_plane_display = NULL;

CSceneRecog::CSceneRecog ()
{
  L_width_ = 0;
  L_height_ = 0;
  p_plane_display = this;
  plane_parameters_[0] = 0.0;
  plane_parameters_[1] = 0.0;
  plane_parameters_[2] = 0.0;
  plane_parameters_[3] = 0.0;

}

CSceneRecog::~CSceneRecog (void)
{
}

void CSceneRecog::srProcess (std::vector < std::vector < CPoint3d > >&data)
{
  laser_data_.swap (data);      // copy the data
  srPlaneExtract ();            // data processing
  srStructureExtract ();
}

void CSceneRecog::srPlaneExtract ()
{
  CPoint3d PointInSquare[DIMEN][DIMEN];
  CPoint3d Mv;
  CPoint3d Nvec;
  int flagbuf;
  int pnum = -1;
  if (laser_data_.size () > 0)
  {
    L_width_ = (int) laser_data_.size ();
    L_height_ = (int) laser_data_[0].size ();

    std::vector < CPointTag > TagVector (L_height_);
    std::vector < std::vector < CPointTag > >PTV (L_width_, TagVector);
    point_tag_vector_.swap (PTV);

    for (int i = 0; i < L_width_ - DIMEN; i = i + 1)
    {
      for (int j = 0; j < L_height_ - DIMEN; j = j + 1)
      {
        Mv.x = Mv.y = Mv.z = 0.0;
        for (int k = 0; k < DIMEN; k++) //get the fitting small planes
          for (int l = 0; l < DIMEN; l++)
          {
            PointInSquare[k][l] = laser_data_[i + k][j + l];
            Mv.x += PointInSquare[k][l].x;
            Mv.y += PointInSquare[k][l].y;
            Mv.z += PointInSquare[k][l].z;
          }
        Mv.x = Mv.x / (DIMEN * DIMEN);  // calculate the barycenter of small planes
        Mv.y = Mv.y / (DIMEN * DIMEN);
        Mv.z = Mv.z / (DIMEN * DIMEN);

        if (srIsAPlane (PointInSquare, Mv))
        {
          Nvec =srNormalVec (laser_data_[i + DIMEN - 1][j + DIMEN - 1], laser_data_[i + DIMEN - 1][j],
                             laser_data_[i][j + DIMEN - 1], laser_data_[i][j]);

          flagbuf = -1;
          for (int f = 0; f <= DIMEN - 1; f++)
            for (int g = 0; g <= DIMEN - 1; g++)
            {
              if (point_tag_vector_[i + f][j + g].pflag != -1)
              {
                flagbuf = point_tag_vector_[i + f][j + g].pflag;
                break;
              }
            }

          if (flagbuf == -1)
          {
            pnum++;
            for (int m = 0; m < DIMEN; m++)
            {
              for (int n = 0; n < DIMEN; n++)
              {
                point_tag_vector_[i + m][j + n].pflag = pnum;
              }
            }
            Plane pTag;
            pTag.VecNum = 1;
            pTag.VecSum = Nvec;
            pTag.VecAve = Nvec;
            pTag.MeanVal = Mv;
            pTag.MeanSum = Mv;
            plane_vector_.push_back (pTag);
          }
          //there are points coincided in adjacent cells
          else
          {
            if (srAngle (plane_vector_[flagbuf].VecAve, Nvec) < 0.2 /*4*M_PI/180 */ )
            {
              for (int m2 = 0; m2 < DIMEN; m2++)
                for (int n2 = 0; n2 < DIMEN; n2++)
                {
                  point_tag_vector_[i + m2][j + n2].pflag = flagbuf;
                }

              plane_vector_[flagbuf].VecNum++;
              //calculate the sum of normals
              plane_vector_[flagbuf].VecSum.x += Nvec.x;
              plane_vector_[flagbuf].VecSum.y += Nvec.y;
              plane_vector_[flagbuf].VecSum.z += Nvec.z;
              //calculate the normals
              plane_vector_[flagbuf].VecAve.x = plane_vector_[flagbuf].VecSum.x / plane_vector_[flagbuf].VecNum;
              plane_vector_[flagbuf].VecAve.y = plane_vector_[flagbuf].VecSum.y / plane_vector_[flagbuf].VecNum;
              plane_vector_[flagbuf].VecAve.z = plane_vector_[flagbuf].VecSum.z / plane_vector_[flagbuf].VecNum;
              //calculate the sum of barycenter
              plane_vector_[flagbuf].MeanSum.x += Mv.x;
              plane_vector_[flagbuf].MeanSum.y += Mv.y;
              plane_vector_[flagbuf].MeanSum.z += Mv.z;
              //calculate the barycenter
              plane_vector_[flagbuf].MeanVal.x = plane_vector_[flagbuf].MeanSum.x / plane_vector_[flagbuf].VecNum;
              plane_vector_[flagbuf].MeanVal.y = plane_vector_[flagbuf].MeanSum.y / plane_vector_[flagbuf].VecNum;
              plane_vector_[flagbuf].MeanVal.z = plane_vector_[flagbuf].MeanSum.z / plane_vector_[flagbuf].VecNum;
            }
          }
        }
      }
    }
  }
}

void CSceneRecog::srStructureExtract () //recover scene frame in indoor scene
{

  using namespace std;

  for (int i = 0; i < L_height_; i++)
  {
    for (int j = 0; j < L_width_; j++)
    {
      int flag = point_tag_vector_[j][i].pflag;
      if (flag >= 0)
      {
        CPointIndex index (i, j);
        plane_vector_[flag].InnerPoints.push_back (index);
      }
    }
  }


  // sort by plane size descending
  int PlaneNum = (int) plane_vector_.size ();
  for (int i = 0; i < PlaneNum - 1; i++)
  {
    int pmax = i;
    for (int j = i + 1; j < PlaneNum; j++)
    {
      if (plane_vector_[j].VecNum > plane_vector_[pmax].VecNum)
      {
        pmax = j;
      }
    }
    if (pmax != i)
      std::swap (plane_vector_[i], plane_vector_[pmax]);
  }
/*
for(int i = 0; i<PlaneNum; i++)
{
	cout<< plane_vector_[i].VecNum<< "  ";
	if((i+1)%20 == 0) cout<<endl;
}*/


  int iterate = 2;
  while (iterate-- > 1)
  {
    //concolidate planes
    int cnt = 0;
    vector < Plane > Planes;    //save the big plane
    vector < Plane > ExcludePlane;
    for (int i = 0; i < (int) plane_vector_.size (); i++)
    {
      cnt++;
      if (plane_vector_[i].VecNum < 4)
        break;
      // calculate the plane equation
      plane_vector_[i].A = plane_vector_[i].VecAve.x;
      plane_vector_[i].B = plane_vector_[i].VecAve.y;
      plane_vector_[i].C = plane_vector_[i].VecAve.z;
      plane_vector_[i].D = -(plane_vector_[i].A * plane_vector_[i].MeanVal.x 
                           + plane_vector_[i].B * plane_vector_[i].MeanVal.y 
                           + plane_vector_[i].C * plane_vector_[i].MeanVal.z);
      // consolidate planes to big planes
      bool flag = false;
      float angle;
      for (int j = 0; j < (int) Planes.size (); j++)
      {
        angle = srAngle (plane_vector_[i].VecAve, Planes[j].VecAve);
        angle = angle < (PI - angle) ? angle : (PI - angle);
        if (angle < 20 * PI / 180)
        {
          if (srP2Pl (plane_vector_[i].MeanVal, Planes[j]) < 0.1)
          {
            Planes[j].VecNum += plane_vector_[i].VecNum;

            //calculate the sum of normals
            Planes[j].VecSum += plane_vector_[i].VecSum;

            // calculate the normals
            Planes[j].VecAve.x = Planes[j].VecSum.x / Planes[j].VecNum;
            Planes[j].VecAve.y = Planes[j].VecSum.y / Planes[j].VecNum;
            Planes[j].VecAve.z = Planes[j].VecSum.z / Planes[j].VecNum;

            // calculate the sum of barycenters
            Planes[j].MeanSum += plane_vector_[i].MeanSum;

            // calculate the sum of barycenters
            Planes[j].MeanVal.x = Planes[j].MeanSum.x / Planes[j].VecNum;
            Planes[j].MeanVal.y = Planes[j].MeanSum.y / Planes[j].VecNum;
            Planes[j].MeanVal.z = Planes[j].MeanSum.z / Planes[j].VecNum;

            Planes[j].A = Planes[j].VecAve.x;
            Planes[j].B = Planes[j].VecAve.y;
            Planes[j].C = Planes[j].VecAve.z;
            Planes[j].D = -(Planes[j].A * Planes[j].MeanVal.x 
                          + Planes[j].B * Planes[j].MeanVal.y 
                          + Planes[j].C * Planes[j].MeanVal.z);

            int I = (int) plane_vector_[i].InnerPoints.size ();
            for (int k = 0; k < I; k++)
            {
              Planes[j].InnerPoints.push_back (plane_vector_[i].InnerPoints[k]);
            }
            flag = true;
            break;
          }

        }
      }
      // if any plane don't recruit this small plane
      if (!flag)
      {
        if ((int) Planes.size () < iterate * iterate * MAX_PLANE_NUM)
        {
          Plane ptemp;
          Planes.push_back (ptemp);
          std::swap (Planes[Planes.size () - 1], plane_vector_[i]);
        }
        else
        {
          Plane ptemp;
          ExcludePlane.push_back (ptemp);
          std::swap (ExcludePlane[ExcludePlane.size () - 1], plane_vector_[i]);
        }
      }
    }
    plane_vector_.swap (Planes);        //save the planes in the vector,and the first element is the biggest plane.
  }
  //find the boundary of every plane
  boundary_vec_.resize (plane_vector_.size ());
  for (int i = 0; i < (int) plane_vector_.size (); i++)
  {
    int flag = 0;
    boundary_vec_[i].push_back (plane_vector_[i].InnerPoints[0]);
    for (int j = 1; j < (int) plane_vector_[i].InnerPoints.size (); j++)
    {
      flag = 1;
      if (plane_vector_[i].InnerPoints[j - 1].i == plane_vector_[i].InnerPoints[j].i
          && plane_vector_[i].InnerPoints[j - 1].j + 1 == plane_vector_[i].InnerPoints[j].j)
        continue;
      else
      {
        boundary_vec_[i].push_back (plane_vector_[i].InnerPoints[j - 1]);
        boundary_vec_[i].push_back (plane_vector_[i].InnerPoints[j]);
        flag = 0;
      }
    }
    if (flag == 1)
      boundary_vec_[i].push_back (plane_vector_[i].InnerPoints.back ());
  }

  /*for(int i = 0; i<plane_vector_.size(); i++)
     {
     cout<< plane_vector_[i].InnerPoints.size()<< " --> "<< boundary_vec_[i].size()<< endl;
     } */

  // calculate the convex hull of every plane
  convex_vec_.resize (boundary_vec_.size ());
  for (int i = 0; i < (int) boundary_vec_.size (); i++)
  {
    int minidx = 0;
    for (int j = 1; j < (int) boundary_vec_[i].size (); j++)
    {
      if (boundary_vec_[i][j].i < boundary_vec_[i][minidx].i
          || (boundary_vec_[i][j].i == boundary_vec_[i][minidx].i
          && boundary_vec_[i][j].j < boundary_vec_[i][minidx].j))
        minidx = j;
    }
    if (minidx != 0)
      std::swap (boundary_vec_[i][0], boundary_vec_[i][minidx]);

    for (int j = 1; j < (int) boundary_vec_[i].size () - 1; j++)
    {
      int r = j;
      for (int k = j + 1; k < (int) boundary_vec_[i].size (); k++)
      {
        bool res = srCompL1R0bool (boundary_vec_[i][0], boundary_vec_[i][r], boundary_vec_[i][k]);
        if (res == 0)
          r = k;
      }
      if (r != j)
        std::swap (boundary_vec_[i][j], boundary_vec_[i][r]);
    }

    std::vector < CPointIndex > stack;
    stack.clear ();
    stack.push_back (boundary_vec_[i][0]);
    stack.push_back (boundary_vec_[i][1]);
    stack.push_back (boundary_vec_[i][2]);

    int top = 2;
    for (int j = 3; j < (int) boundary_vec_[i].size (); j++)
    {
      while (top >= 2 && srCross_Product (stack[top - 1], stack[top], boundary_vec_[i][j]) <= 0)
      {
        stack.pop_back ();
        top--;
      }
      stack.push_back (boundary_vec_[i][j]);
      top++;
    }
    convex_vec_[i].swap (stack);
  }
  /*
     for(int i = 0; i<boundary_vec_.size(); i++)
     {
     cout<< boundary_vec_[i].size() << " --> "<< boundary_vec_[i].size() << endl;
     }
   */
  plane_parameters_[0] = plane_vector_[0].A;
  plane_parameters_[1] = plane_vector_[0].B;
  plane_parameters_[2] = plane_vector_[0].C;
  plane_parameters_[3] = plane_vector_[0].D;
  //printf("%f,%f,%f,%f\n",planeparameters[0],planeparameters[1],planeparameters[2],planeparameters[3]);

  /*ofstream w("plane.txt");
     if (w.is_open())
     {
     printf("open the file ok\n");
     }
     else
     {
     printf("open the file fail\n");
     return 0 ;
     }
     for (int i = 0; i < plane_vector_[0].InnerPoints.size();i++)
     {
     int x = plane_vector_[0].InnerPoints[i].i;
     int y = plane_vector_[0].InnerPoints[i].j;
     w<<laser_data_[x][y].x<<"  "<<laser_data_[x][y].y<<"  "<<laser_data_[x][y].z<<"\n";
     }
     w.close(); */

  //return plane_parameters_;

}

float CSceneRecog::srPtopDistan (CPoint3d p1, CPoint3d p2)
{
  return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

bool CSceneRecog::srIsAPlane (CPoint3d p[][DIMEN], CPoint3d m)
{
  CPoint3d Pzero (0, 0, 0);
  int n = 3, jt = 100;
  float cov[9] = { 0.0 };
  float s1, s2, s3, min;
  float eps = 0.01f;
  CPoint3d sum, ptemp;
  int zeronum = 0;
  bool booldis = true;
  float distemp, maxdis = 0.0;

  ptemp = p[0][0];
  for (int i = 0; i < DIMEN; i++)
  {
    for (int j = 1; j < DIMEN; j++)
    {
      if (Pzero == p[i][j])
      {
        zeronum++;
      }
      else
      {
        distemp = srPtopDistan (ptemp, p[i][j]);
        if (distemp > maxdis)
        {
          maxdis = distemp;
        }
        ptemp = p[i][j];
      }
    }
  }

  if (maxdis > MAXDISTH1 * MAXDISTH1)   // || zeronum>DIMEN                
  {
    booldis = false;
    return false;
  }
  for (int i = 0; i < DIMEN; i++)
  {
    for (int j = 0; j < DIMEN; j++)
    {
      float buf[3];
      buf[0] = p[i][j].x - m.x;
      buf[1] = p[i][j].y - m.y;
      buf[2] = p[i][j].z - m.z;
      for (int k = 0; k < 3; k++)
        for (int l = 0; l < 3; l++)
          if (!(p[i][j] == Pzero))
          {
            cov[k * 3 + l] += buf[k] * buf[l];
          }
    }
  }
  // calculate the covariance matrice eigenvalue
  srEigenValue (cov, n, eps, jt);
  s1 = fabs (cov[0]);
  s2 = fabs (cov[4]);
  s3 = fabs (cov[8]);
  min = s1 < s2 ? s1 : s2;
  min = s3 < min ? s3 : min;

  //compare the minimum of eigenvalue with threshold
  if (min < THRESH && zeronum < DIMEN && booldis)
    return true;
  return false;
}

int CSceneRecog::srEigenValue (float a[], int n, double eps, int jt)
{
  int i, j, u, w, t, s, l;
  int p = 0;
  int q = 0;
  float fm, cn, sn, omega, x, y, d, v[30];
  l = 1;
  for (i = 0; i <= n - 1; i++)
  {
    v[i * n + i] = 1.0;
    for (j = 0; j <= n - 1; j++)
    {
      if (i != j)
      {
        v[i * n + j] = 0.0;
      }
    }
  }
  while (l == 1)
  {
    fm = 0.0;
    for (i = 0; i <= n - 1; i++)
    {
      for (j = 0; j <= n - 1; j++)
      {
        d = fabs (a[i * n + j]);
        if ((i != j) && (d > fm))
        {
          fm = d;
          p = i;
          q = j;
        }
      }
    }
    if (fm < eps)
    {
      return (1);
    }
    if (l > jt)
    {
      return (-1);
    }
    l = l + 1;
    u = p * n + q;
    w = p * n + p;
    t = q * n + p;
    s = q * n + q;
    x = -a[u];
    y = (a[s] - a[w]) / 2.0;
    omega = x / sqrt (x * x + y * y);
    if (y < 0.0)
    {
      omega = -omega;
    }
    sn = 1.0 + sqrt (1.0 - omega * omega);
    sn = omega / sqrt (2.0 * sn);
    cn = sqrt (1.0 - sn * sn);
    fm = a[w];
    a[w] = fm * cn * cn + a[s] * sn * sn + a[u] * omega;
    a[s] = fm * sn * sn + a[s] * cn * cn - a[u] * omega;
    a[u] = 0.0;
    a[t] = 0.0;
    for (j = 0; j <= n - 1; j++)
    {
      if ((j != p) && (j != q))
      {
        u = p * n + j;
        w = q * n + j;
        fm = a[u];
        a[u] = fm * cn + a[w] * sn;
        a[w] = -fm * sn + a[w] * cn;
      }
    }
    for (i = 0; i <= n - 1; i++)
    {
      if ((i != p) && (i != q))
      {
        u = i * n + p;
        w = i * n + q;
        fm = a[u];
        a[u] = fm * cn + a[w] * sn;
        a[w] = -fm * sn + a[w] * cn;
      }
    }
    for (i = 0; i <= n - 1; i++)
    {
      u = i * n + p;
      w = i * n + q;
      fm = v[u];
      v[u] = fm * cn + v[w] * sn;
      v[w] = -fm * sn + v[w] * cn;
    }
  }
  return 0;
}

CPoint3d CSceneRecog::srNormalVec (CPoint3d start1, CPoint3d end1, CPoint3d start2, CPoint3d end2)
{
  float r1, r2;
  CPoint3d point1, point2, point;
  point1 = srVecMuti (start1, end1, start1, end2);
  point2 = srVecMuti (start2, end1, start2, end2);
  r1 = sqrt (point1.x * point1.x + point1.y * point1.y + point1.z * point1.z);
  r2 = sqrt (point2.x * point2.x + point2.y * point2.y + point2.z * point2.z);
  point.x = (point1.x / r1 + point2.x / r2) / 2;
  point.y = (point1.y / r1 + point2.y / r2) / 2;
  point.z = (point1.z / r1 + point2.z / r2) / 2;
  return point;
}

CPoint3d CSceneRecog::srVecMuti (CPoint3d start1, CPoint3d end1, CPoint3d start2, CPoint3d end2)
{
  float x1, y1, z1;
  float x2, y2, z2;
  x1 = end1.x - start1.x;
  y1 = end1.y - start1.y;
  z1 = end1.z - start1.z;
  x2 = end2.x - start2.x;
  y2 = end2.y - start2.y;
  z2 = end2.z - start2.z;
  CPoint3d point;
  point.x = y1 * z2 - y2 * z1;
  point.y = x2 * z1 - x1 * z2;
  point.z = x1 * y2 - x2 * y1;
  return point;
}

float CSceneRecog::srAngle (CPoint3d vec1, CPoint3d vec2)
{
  float x1, y1, z1, p1, p2, /*r1,r2, */ result, theta;
  x1 = vec1.x * vec2.x;
  y1 = vec1.y * vec2.y;
  z1 = vec1.z * vec2.z;
  p1 = vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z;
  p2 = vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z;
  result = (x1 + y1 + z1) / sqrt (p1 * p2);
  theta = acos (result);
  return theta;
}

float CSceneRecog::srP2Pl (CPoint3d m, const Plane & Plane)
{
  float dist;
  dist = fabs (Plane.A * m.x + Plane.B * m.y + Plane.C * m.z + Plane.D);
  dist = dist / sqrt (Plane.A * Plane.A + Plane.B * Plane.B + Plane.C * Plane.C);
  return dist;
}

void CSceneRecog::srCalColor (int idx, std::vector < float >&rgb)
{
  static int preidx = 1000;
  float step = 1;
  if (idx < 0)
  {
    rgb[0] = 0.5;
    rgb[1] = 0.5;
    rgb[2] = 0.5;
  }
  else if (idx != preidx)
  {
    float ratio = idx % 8;
    ratio = (1 - ratio / 8);
    switch (idx % 6)
    {
      case 0:
        rgb[0] = ratio * step;
        rgb[1] = 0;
        rgb[2] = 0;
        break;
      case 1:
        rgb[0] = 0;
        rgb[1] = ratio * step;
        rgb[2] = 0;
        break;
      case 2:
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = ratio * step;
        break;
      case 3:
        rgb[0] = ratio * step;
        rgb[1] = ratio * step;
        rgb[2] = 0;
        break;
      case 4:
        rgb[0] = ratio * step;
        rgb[1] = 0;
        rgb[2] = ratio * step;
        break;
      case 5:
        rgb[0] = 0;
        rgb[1] = ratio * step;
        rgb[2] = ratio * step;
        break;
    }
  }
  preidx = idx;
}

bool CSceneRecog::srCompL1R0bool (const CPointIndex & p1, const CPointIndex & p2, const CPointIndex & p3)
{
  bool res = 1;
  int temp = (p2.j - p1.j) * (p3.i - p1.i) - (p2.i - p1.i) * (p3.j - p1.j);
  if (temp < 0)
    res = 0;
  else if (temp == 0)
  {
    int dist1 = (p2.i - p1.i) * (p2.i - p1.i) + (p2.j - p1.j) * (p2.j - p1.j);
    int dist2 = (p3.i - p1.i) * (p3.i - p1.i) + (p3.j - p1.j) * (p3.j - p1.j);
    if (dist1 > dist2)
      res = 0;
  }
  return res;
}

inline int CSceneRecog::srCross_Product (const CPointIndex & p1, const CPointIndex & p2, const CPointIndex & p3)
{
  return (p2.j - p1.j) * (p3.i - p1.i) - (p2.i - p1.i) * (p3.j - p1.j);
}
