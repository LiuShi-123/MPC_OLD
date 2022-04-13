#include "SomeFunc.h"
Eigen::MatrixXd _SomeFunc::Power(Eigen::MatrixXd A, int power)
{
  int t = 1;
  Eigen::MatrixXd power_Res = A;
  if (power == 0) {
    power_Res = Eigen::MatrixXd::Identity(_SomeFunc::prs.Nx + _SomeFunc::prs.Nu, _SomeFunc::prs.Nx + _SomeFunc::prs.Nu);
  } else if (power == 1) {
    power_Res = A;
  } else {
    while (t < power) {
      power_Res = power_Res * A;
      t++;
    }
  }
  return power_Res;
}

Eigen::MatrixXd _SomeFunc::give_PHI(Eigen::MatrixXd C, Eigen::MatrixXd A)
{
  int t = 0;
  int end = _SomeFunc::prs.Np;
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nx * _SomeFunc::prs.Np, _SomeFunc::prs.Nx + _SomeFunc::prs.Nu);
  Eigen::MatrixXd temp_Power = A;
  while (t < end) {
    temp_Power = _SomeFunc::Power(A, t + 1);
    res.block(_SomeFunc::prs.Nx * t, 0, _SomeFunc::prs.Nx, _SomeFunc::prs.Nx + _SomeFunc::prs.Nu) = C * temp_Power;
    t++;
  }
  return res;
}

Eigen::MatrixXd _SomeFunc::give_Qieta(Eigen::MatrixXd C, Eigen::MatrixXd A, Eigen::MatrixXd B)
{
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nx * _SomeFunc::prs.Np, _SomeFunc::prs.Nu * _SomeFunc::prs.Nc);
  int h = 0;
  int l = 0;
  int power_A = 0;
  Eigen::MatrixXd temp_Block = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nx, _SomeFunc::prs.Nu);
  Eigen::MatrixXd temp_Power = A;
  while (l < _SomeFunc::prs.Nc) {
    h = 0;
    power_A = 0;
    while (h < _SomeFunc::prs.Np) {
      if (h < l) {
        temp_Block = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nx, _SomeFunc::prs.Nu);
      } else {
        temp_Power = _SomeFunc::Power(A, power_A);
        temp_Block = C * temp_Power;
        temp_Block = temp_Block * B;
        power_A++;
      }
      res.block(h * _SomeFunc::prs.Nx, l * _SomeFunc::prs.Nu, _SomeFunc::prs.Nx, _SomeFunc::prs.Nu) = temp_Block;
      h++;
    }
    l++;
  }
  return res;
}

Eigen::VectorXd _SomeFunc::turn_Vec(Eigen::MatrixXd matrix)
{
  int h = matrix.rows();
  Eigen::VectorXd res(h);
  int ht = 0;
  while (ht < h) {
    res(ht) = matrix(ht, 0);
    ht++;
  }
  return res;
}

Eigen::SparseMatrix<double> _SomeFunc::give_AI()
{
  int h = 0;
  int l = 0;
  Eigen::MatrixXd tempZero = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nu, _SomeFunc::prs.Nu);
  Eigen::MatrixXd tempIden = Eigen::MatrixXd::Identity(_SomeFunc::prs.Nu, _SomeFunc::prs.Nu);
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nc * _SomeFunc::prs.Nu, _SomeFunc::prs.Nc * _SomeFunc::prs.Nu + 1);//change
  while (l < _SomeFunc::prs.Nc) {
    h = 0;
    while (h < _SomeFunc::prs.Nc) {
      if (h < l) {
      res.block(h * _SomeFunc::prs.Nu, l * _SomeFunc::prs.Nu, _SomeFunc::prs.Nu, _SomeFunc::prs.Nu) = tempZero;
      } else if(h >= l){
      res.block(h * _SomeFunc::prs.Nu, l * _SomeFunc::prs.Nu, _SomeFunc::prs.Nu, _SomeFunc::prs.Nu) = tempIden;
      }
      h++;
    }
    l++;
  }
  return res.sparseView();
}
Eigen::VectorXd _SomeFunc::give_lb_ub(Eigen::MatrixXd kesi, int i)
{
  Eigen::MatrixXd Ut = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nu * _SomeFunc::prs.Nc, 1);
  Eigen::MatrixXd um = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nu, 1);
  Eigen::MatrixXd Umax = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nu * _SomeFunc::prs.Nc, 1);
  int h = 0;
  Eigen::MatrixXd uk1 = Eigen::MatrixXd::Zero(_SomeFunc::prs.Nu, 1);
  um << _SomeFunc::prs.Umax0,
      _SomeFunc::prs.Umax1;
  uk1 << kesi(_SomeFunc::prs.Nx, 0),
      kesi(_SomeFunc::prs.Nx + 1, 0);
  while (h < _SomeFunc::prs.Nc) {
    Ut.block(_SomeFunc::prs.Nu * h, 0, _SomeFunc::prs.Nu, 1) = uk1;
    h++;
  }
  h = 0;
  while (h < _SomeFunc::prs.Nc) {
    Umax.block(_SomeFunc::prs.Nu * h, 0, _SomeFunc::prs.Nu, 1) = um;
    h++;
  }
  if (i == 1) {
    return _SomeFunc::turn_Vec(Umax - Ut);
  } else {
    return _SomeFunc::turn_Vec(-Ut - Umax);
  }
}
