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
#include "camera_laser_calibration/white_black_grid.h"

CWhiteBlackGrid::CWhiteBlackGrid (void)
{
  scene_cnt_ = 0;
  m_scan_mode_ = 1;
}

CWhiteBlackGrid::~CWhiteBlackGrid (void)
{
}

void CWhiteBlackGrid::whgRegisterScanMode (bool scan_mode)
{
  m_scan_mode_ = scan_mode;
}

void CWhiteBlackGrid::whgNodeDetect (int _m, int _n, float *_parameters,
                                     std::vector < std::vector < CPoint3d > >&laser_data_vec)
{
  size_t k_num;
  k_num = (size_t) _m *_n;

  //3D points binaryzation .
  size_t m, n;
  m = _m;
  n = _n;
  float A = _parameters[0];
  float B = _parameters[1];
  float C = _parameters[2];
  float D = _parameters[3];
  bin_vec_.resize (m);
  for (size_t p = 0; p < m; p++)
  {
    bin_vec_[p].resize (n);
    for (size_t q = 0; q < n; q++)
    {
      double d =
        abs (laser_data_vec[p][q].x * A + laser_data_vec[p][q].y * B + laser_data_vec[p][q].z * C + D) / sqrt (A * A +
                                                                                                               B * B +
                                                                                                               C * C);
      if (d > 0.15)
      {
        bin_vec_[p][q] = 0;
      }
      else
        bin_vec_[p][q] = 255;
    }
  }

  int iChoosed = 0;
  bool isFound = false;

  for (size_t i = 0; i < n; i++)
  {
    for (size_t j = 0; j < m; j++)
    {
      if (bin_vec_[j][i] == 255)
      {
        isFound = true;
        continue;
      }
      else
      {
        isFound = false;
        break;
      }
    }
    if (isFound)
    {
      iChoosed = i;
      break;
    }
  }

  std::vector < int >Bin_ (n - iChoosed);
  std::vector < std::vector < int > >BinNew (m);
  std::vector < CPoint3d > LaserOne (n - iChoosed);
  std::vector < std::vector < CPoint3d > >LaserData_ (m);

  for (size_t j = 0; j < m; j++)
  {
    for (size_t i = iChoosed, i_ = 0; i < n; i++, i_++)
    {
      Bin_[i_] = bin_vec_[j][i];
      LaserOne[i_] = laser_data_vec[j][i];
    }
    BinNew[j] = Bin_;
    LaserData_[j] = LaserOne;
  }
  bin_vec_.swap (BinNew);
  laser_data_vec.swap (LaserData_);
  n = n - iChoosed;
  /*ofstream ww("bin_vec_.txt");
     if (!ww.is_open())
     {
     printf("open the file bin_vec_.txt fail\n");
     return ;
     }
     else
     printf("open the file bin_vec_.txt ok\n");
     for (int i = 0; i < m; i++)
     {
     for (int j = 0; j < n;j++)
     {
     ww<<bin_vec_[i][j]<<"  ";
     }
     ww<<"\n";
     }
     ww.close(); */

  //Binary matrix processing 
  std::vector < BinIndex > centerij;
  BinIndex tmpindex;
  size_t rem;

  size_t DELTR;
  size_t DELTL;

  if (m_scan_mode_)             //pitching scan
  {
    rem = m % 4;
    if (rem != 0)
    {
      m = m - rem;
    }

    rem = n % 5;
    if (rem != 0)
    {
      n = n - rem;
    }
    DELTR = m / 4;
    DELTL = n / 5;
  }
  else                          //horizontal scan
  {
    rem = m % 5;
    if (rem != 0)
    {
      m = m - rem;
    }

    rem = n % 4;
    if (rem != 0)
    {
      n = n - rem;
    }
    DELTR = m / 5;
    DELTL = n / 4;
  }

  std::vector < size_t > rownum;        //save the number of '0' for each row 
  rownum.resize (m);
  std::vector < size_t > linenum;       //save the number of '0' for each column
  linenum.resize (n);
  std::vector < size_t > maxindex;
  //solution 1
  for (size_t p = 0; p < m; p += DELTR)
  {
    for (size_t q = 0; q < n; q += DELTL)
    {
      for (size_t j = 0; j < m; j++)
      {
        rownum[j] = 0;
      }
      for (size_t i = 0; i < n; i++)
      {
        linenum[i] = 0;
      }

      //get the number of '0' for each row
      for (size_t j = p; j < p + DELTR; j++)
      {
        for (size_t i = q; i < q + DELTL; i++)
        {
          if ((bin_vec_[j][i] == 0))
          {
            rownum[j] += 1;
          }
        }
      }
      //get the maximum of row number
      size_t maxj = p, maxvaluej = 0;
      for (size_t j = p; j < p + DELTR; j++)
      {
        if (rownum[j] >= maxvaluej)
        {
          maxvaluej = rownum[j];
          maxj = j;
        }
      }
      //if there are many maximum of row number
      std::vector < size_t > vec_ (0);
      maxindex.swap (vec_);
      for (size_t j = p; j < p + DELTR; j++)
      {
        if (rownum[j] == maxvaluej)
          maxindex.push_back (j);
      }
      size_t size_maxj = (size_t) maxindex.size ();
      size_t summaxj = 0;
      for (size_t k = 0; k < size_maxj; k++)
      {
        summaxj += maxindex[k];
      }
      float quo;
      size_t avemaxj;
      quo = summaxj / size_maxj;
      if ((quo - (size_t) quo) < 0.5)
        avemaxj = (size_t) quo;
      else
        avemaxj = (size_t) quo + 1;
      //get the number of '0' for each column
      for (size_t i = q; i < q + DELTL; i++)
      {
        for (size_t j = p; j < p + DELTR; j++)
        {
          if ((bin_vec_[j][i] == 0))
          {
            linenum[i] += 1;
          }
        }
      }

      //get the maximum of row number
      size_t maxi = q, maxvaluei = 0;
      for (size_t i = q; i < q + DELTL; i++)
      {
        if (linenum[i] >= maxvaluei)
        {
          maxvaluei = linenum[i];
          maxi = i;
        }
      }

      //if there are many maximum of row number
      maxindex.clear ();
      for (size_t i = q; i < q + DELTL; i++)
      {
        if (linenum[i] == maxvaluei)
          maxindex.push_back (i);
      }
      size_t size_maxi = (size_t) maxindex.size ();
      size_t summaxi = 0;
      for (size_t k = 0; k < size_maxi; k++)
      {
        summaxi += maxindex[k];
      }
      size_t avemaxi;
      quo = summaxi / size_maxi;
      if ((quo - (size_t) quo) < 0.5)
        avemaxi = (size_t) quo;
      else
        avemaxi = (size_t) quo + 1;



      tmpindex.bin_j = avemaxj;
      tmpindex.bin_i = avemaxi;
      centerij.push_back (tmpindex);
    }
  }

  std::vector < CPoint3d > Node_ (0);
  node_vec_.swap (Node_);
  if (m_scan_mode_)             //pitching scan
  {
    for (int k = 0; k < 3; k++)
    {
      for (int i = 5 * k; i < 5 * k + 4; i++)
      {
        BinIndex avenode = whgAverageof4index (centerij[i], centerij[i + 1], centerij[i + 5], centerij[i + 6]);
        CPoint3d ptmp;
        ptmp.x = laser_data_vec[avenode.bin_j][avenode.bin_i].x;
        ptmp.y = laser_data_vec[avenode.bin_j][avenode.bin_i].y;
        ptmp.z = laser_data_vec[avenode.bin_j][avenode.bin_i].z;
        node_vec_.push_back (ptmp);
      }
    }
  }
  else                          //horizontal scan
  {
    for (int k = 0; k < 4; k++)
    {
      for (int i = 4 * k; i < 4 * k + 3; i++)
      {
        BinIndex avenode = whgAverageof4index (centerij[i], centerij[i + 1], centerij[i + 4], centerij[i + 5]);
        CPoint3d ptmp;
        ptmp.x = laser_data_vec[avenode.bin_j][avenode.bin_i].x;
        ptmp.y = laser_data_vec[avenode.bin_j][avenode.bin_i].y;
        ptmp.z = laser_data_vec[avenode.bin_j][avenode.bin_i].z;
        node_vec_.push_back (ptmp);
      }
    }

  }
}

// Calculate the average of 4 indexs
BinIndex CWhiteBlackGrid::whgAverageof4index (BinIndex bi1, BinIndex bi2, BinIndex bi3, BinIndex bi4)
{
  int av_j, av_i, sum_i = 0, sum_j = 0;
  sum_j = bi1.bin_j + bi2.bin_j + bi3.bin_j + bi4.bin_j;
  sum_i = bi1.bin_i + bi2.bin_i + bi3.bin_i + bi4.bin_i;
  float quoj, quoi;
  quoj = sum_j / 4;
  if ((quoj - (int) quoj) < 0.5)
  {
    av_j = (int) quoj;
  }
  else
  {
    av_j = (int) quoj + 1;
  }

  quoi = sum_i / 4;
  if ((quoi - (int) quoi) < 0.5)
  {
    av_i = (int) quoi;
  }
  else
  {
    av_i = (int) quoi + 1;
  }
  BinIndex tmp;
  tmp.bin_j = av_j;
  tmp.bin_i = av_i;
  return (tmp);
}

void CWhiteBlackGrid::whgNodeModify (void)      //Revise the corners we detect.
{
  int nump = (int) node_vec_.size ();
  match_pair_vec_.resize (nump);
  for (int i = 0; i < nump; i++)
  {
    match_pair_vec_[i].x = node_vec_[i].x;
    match_pair_vec_[i].y = node_vec_[i].y;
    match_pair_vec_[i].z = node_vec_[i].z;
  }

  standard_vec_.resize (12);
  if (m_scan_mode_)             //pitching scan
  {
    standard_vec_[0] = CPoint3d (0.75, 0.00, 0);
    standard_vec_[1] = CPoint3d (0.50, 0.00, 0);
    standard_vec_[2] = CPoint3d (0.25, 0.00, 0);
    standard_vec_[3] = CPoint3d (0, 0.00, 0);

    standard_vec_[4] = CPoint3d (0.75, 0.25, 0);
    standard_vec_[5] = CPoint3d (0.50, 0.25, 0);
    standard_vec_[6] = CPoint3d (0.25, 0.25, 0);
    standard_vec_[7] = CPoint3d (0, 0.25, 0);

    standard_vec_[8] = CPoint3d (0.75, 0.50, 0);
    standard_vec_[9] = CPoint3d (0.50, 0.50, 0);
    standard_vec_[10] = CPoint3d (0.25, 0.50, 0);
    standard_vec_[11] = CPoint3d (0, 0.50, 0);
  }
  else                          //horizontal scan
  {
    standard_vec_[0] = CPoint3d (0, 0, 0);
    standard_vec_[1] = CPoint3d (0, 0.25, 0);
    standard_vec_[2] = CPoint3d (0, 0.50, 0);

    standard_vec_[3] = CPoint3d (0.25, 0, 0);
    standard_vec_[4] = CPoint3d (0.25, 0.25, 0);
    standard_vec_[5] = CPoint3d (0.25, 0.50, 0);

    standard_vec_[6] = CPoint3d (0.50, 0, 0);
    standard_vec_[7] = CPoint3d (0.50, 0.25, 0);
    standard_vec_[8] = CPoint3d (0.50, 0.50, 0);

    standard_vec_[9] = CPoint3d (0.75, 0, 0);
    standard_vec_[10] = CPoint3d (0.75, 0.25, 0);
    standard_vec_[11] = CPoint3d (0.75, 0.50, 0);
  }
  whgCalculateTransformationRT ();      //calculate R £¬t
}

void CWhiteBlackGrid::whgCalculateTransformationRT (void)       //calculate the R ,t
{
  int n = (int) match_pair_vec_.size ();
  VectorXd avr_m (3), avr_d (3);        // Centroid vector 3*1
  MatrixXd matrix_M (n, 3), matrix_D (n, 3);    // raw matrix n*3
  MatrixXd matrix_MM (n, 3), matrix_DD (n, 3);  // 
  MatrixXd matrix_S (3, 3);     // parameter matrix of covariance matrices 3*3
  MatrixXd matrix_N (4, 4);     // Covariance matrices 4*4
  //VectorXd lambda;                                              // eigenvalue
  MatrixXd Q;                   // eigenvector
  VectorXd vector_q (4);        // quaternion q
  VectorXd vector_c;

  MatrixXd matrix_MyR (3, 3);   // Rotation matrix R
  VectorXd vector_Myt (3);      // Translation vector t

  for (int i = 0; i < n; i++)
  {
    matrix_M (i, 0) = match_pair_vec_[i].x;
    matrix_M (i, 1) = match_pair_vec_[i].y;
    matrix_M (i, 2) = match_pair_vec_[i].z;

    matrix_D (i, 0) = standard_vec_[i].x;
    matrix_D (i, 1) = standard_vec_[i].y;
    matrix_D (i, 2) = standard_vec_[i].z;
  }

  avr_m[0] = 0.0;
  avr_m[1] = 0.0;
  avr_m[2] = 0.0;

  avr_d[0] = 0.0;
  avr_d[1] = 0.0;
  avr_d[2] = 0.0;

  for (int i = 0; i < n; i++)
  {
    avr_m[0] += match_pair_vec_[i].x;
    avr_m[1] += match_pair_vec_[i].y;
    avr_m[2] += match_pair_vec_[i].z;

    avr_d[0] += standard_vec_[i].x;
    avr_d[1] += standard_vec_[i].y;
    avr_d[2] += standard_vec_[i].z;
  }
  avr_m[0] /= n;
  avr_m[1] /= n;
  avr_m[2] /= n;

  avr_d[0] /= n;
  avr_d[1] /= n;
  avr_d[2] /= n;

  for (int i = 0; i < n; i++)
  {
    matrix_MM (i, 0) = matrix_M (i, 0) - avr_m[0];
    matrix_MM (i, 1) = matrix_M (i, 1) - avr_m[1];
    matrix_MM (i, 2) = matrix_M (i, 2) - avr_m[2];

    matrix_DD (i, 0) = matrix_D (i, 0) - avr_d[0];
    matrix_DD (i, 1) = matrix_D (i, 1) - avr_d[1];
    matrix_DD (i, 2) = matrix_D (i, 2) - avr_d[2];
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      matrix_S (i, j) = 0.0;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < n; k++)
      {
        matrix_S (i, j) += matrix_MM (k, i) * matrix_DD (k, j);
      }
    }
  }
  // calculate the matrix_N
  matrix_N (0, 0) = matrix_S (0, 0) + matrix_S (1, 1) + matrix_S (2, 2);

  matrix_N (0, 1) = matrix_N (1, 0) = matrix_S (1, 2) - matrix_S (2, 1);
  matrix_N (0, 2) = matrix_N (2, 0) = matrix_S (2, 0) - matrix_S (0, 2);
  matrix_N (0, 3) = matrix_N (3, 0) = matrix_S (0, 1) - matrix_S (1, 0);

  matrix_N (1, 1) = matrix_S (0, 0) - matrix_S (1, 1) - matrix_S (2, 2);

  matrix_N (1, 2) = matrix_N (2, 1) = matrix_S (0, 1) + matrix_S (1, 0);
  matrix_N (1, 3) = matrix_N (3, 1) = matrix_S (2, 0) + matrix_S (0, 2);
  matrix_N (3, 2) = matrix_N (2, 3) = matrix_S (1, 2) + matrix_S (2, 1);

  matrix_N (2, 2) = -matrix_S (0, 0) + matrix_S (1, 1) - matrix_S (2, 2);
  matrix_N (3, 3) = -matrix_S (0, 0) - matrix_S (1, 1) + matrix_S (2, 2);


  // calculate the eigenvalue and eigenvector of matrix_N
  EigenSolver < MatrixXd > es (matrix_N);
  // find the eigenvector of maximum eigenvalue
  double lambda[4] = { 0 };
  for (int i = 0; i < 4; ++i)
  {
    lambda[i] = real (es.eigenvalues ()[i]);
  }
  double max1_ = max (lambda[0], lambda[1]);
  double max2_ = max (lambda[2], lambda[3]);
  double max_ = max (max1_, max2_);
  int j = 0;
  for (; j < 4; j++)
  {
    if (lambda[j] == max_)
    {
      break;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    vector_q[i] = real (es.eigenvectors ().col (j)[i]); //get the eigenvector
  }

  // calculate the matrix_MyR
  matrix_MyR (0, 0) = pow (vector_q[0], 2) + pow (vector_q[1], 2) - pow (vector_q[2], 2) - pow (vector_q[3], 2);
  matrix_MyR (0, 1) = 2 * (vector_q[1] * vector_q[2] - vector_q[3] * vector_q[0]);      //
  matrix_MyR (0, 2) = 2 * (vector_q[1] * vector_q[3] + vector_q[2] * vector_q[0]);

  matrix_MyR (1, 0) = 2 * (vector_q[1] * vector_q[2] + vector_q[3] * vector_q[0]);
  matrix_MyR (1, 1) = pow (vector_q[0], 2) - pow (vector_q[1], 2) + pow (vector_q[2], 2) - pow (vector_q[3], 2);
  matrix_MyR (1, 2) = 2 * (vector_q[2] * vector_q[3] - vector_q[1] * vector_q[0]);

  matrix_MyR (2, 0) = 2 * (vector_q[3] * vector_q[1] - vector_q[2] * vector_q[0]);
  matrix_MyR (2, 1) = 2 * (vector_q[3] * vector_q[2] + vector_q[1] * vector_q[0]);
  matrix_MyR (2, 2) = pow (vector_q[0], 2) - pow (vector_q[1], 2) - pow (vector_q[2], 2) + pow (vector_q[3], 2);

  MatrixXd matrix_MyRR;
  matrix_MyRR = matrix_MyR.inverse ();

  // calculate the vector_Myt
  vector_c = matrix_MyRR * avr_d;
  vector_Myt = avr_m - vector_c;

  whgTransform (matrix_MyRR, vector_Myt, standard_vec_);

  //save these corners
  ofstream outf;
  char filename[100] = "laserpts.txt";
  //sprintf(filename,"laserpts.txt",scene_cnt_);
  outf.open (filename);
  if (outf.fail ())
  {
    cout << "can not open file";
    
    return;
  }

  for (size_t i = 0; i < standard_vec_.size (); i++)
  {
    outf.setf (ios::right);
    outf.width (8);
    outf << standard_vec_[i].x << " " << standard_vec_[i].y << " " << standard_vec_[i].z << endl;
  }
  scene_cnt_++;

  outf.close ();
}

void CWhiteBlackGrid::whgTransform (MatrixXd matrix_R, VectorXd vector_t, std::vector < CPoint3d > &laser_point_vec)
{
  VectorXd vector_c;
  VectorXd vector_d (3);
  int n = (int) laser_point_vec.size ();
  for (int i = 0; i < n; i++)
  {
    vector_d[0] = laser_point_vec[i].x;
    vector_d[1] = laser_point_vec[i].y;
    vector_d[2] = laser_point_vec[i].z;

    // d=R*d+t
    vector_c = matrix_R * vector_d;
    vector_d = vector_c + vector_t;

    laser_point_vec[i].x = vector_d[0];
    laser_point_vec[i].y = vector_d[1];
    laser_point_vec[i].z = vector_d[2];
  }
}
