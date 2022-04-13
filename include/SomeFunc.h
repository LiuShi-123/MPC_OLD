#ifndef _SOMEFUNC
#define _SOMEFUNC
#include "OsqpEigen/OsqpEigen.h"
#include "Paras.h"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
class _SomeFunc {
  public:
  _Paras prs;
  Eigen::MatrixXd Power(Eigen::MatrixXd A, int power);
  Eigen::MatrixXd give_PHI(Eigen::MatrixXd C, Eigen::MatrixXd A);
  Eigen::MatrixXd give_Qieta(Eigen::MatrixXd C, Eigen::MatrixXd A, Eigen::MatrixXd B);
  Eigen::VectorXd turn_Vec(Eigen::MatrixXd matrix);
  Eigen::SparseMatrix<double> give_AI();
  Eigen::VectorXd give_lb_ub(Eigen::MatrixXd kesi, int i);
};
#endif
