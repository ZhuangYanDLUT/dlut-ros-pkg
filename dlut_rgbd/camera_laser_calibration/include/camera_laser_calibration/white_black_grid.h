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
#ifndef WHITE_BLACK_GRID_H
#define WHITE_BLACK_GRID_H

#include "camera_laser_calibration/basic_data/point3d.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <eigen3/Eigen/Dense>   //matirx relation

#define N 3

using namespace std;
using namespace Eigen;


struct BinIndex                 //Binary matrix index struct
{
  size_t bin_i;
  size_t bin_j;
};

class CWhiteBlackGrid
{
public:
	CWhiteBlackGrid (void);
  ~CWhiteBlackGrid (void);
  
  std::vector < std::vector < int > >bin_vec_;   // Binary matrix
  std::vector < CPoint3d > node_vec_; //Checkerboard corners which are detected initially
  std::vector < CPoint3d > match_pair_vec_;   //Checkerboard corners which are detected 
  std::vector < CPoint3d > standard_vec_;     //Constructed standard model 

	void whgRegisterScanMode (bool scan_mode);
  void whgNodeDetect (int _m, int _n, float *_parameters, std::vector < std::vector < CPoint3d > >&laser_data_vec);     // Detect checkerboard corners
  void whgNodeModify (void);    //Revise the corners we detect.

  BinIndex whgAverageof4index (BinIndex bi1, BinIndex bi2, BinIndex bi3, BinIndex bi4); //Calculate the average of 4 indexs

  void whgCalculateTransformationRT (void);     //Calculate the RT
  void whgTransform (MatrixXd matrix_R, VectorXd vector_t, std::vector < CPoint3d > &laser_point_vec);
  
private:
  int scene_cnt_;               //The number of scene
  bool m_scan_mode_;            //scan types 1:pitching,0:horizontal 
};

#endif
