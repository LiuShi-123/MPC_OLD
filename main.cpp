#include "CarModule.h"
#include "OsqpEigen/OsqpEigen.h"
#include "Paras.h"
#include "RefWay.h"
#include "SomeFunc.h"
#include "simConst.h"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <osqp.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
extern "C" {
#include "extApi.h"
}
using namespace std;
using namespace Eigen;
int clientID = simxStart("127.0.0.1", 19999, true, true, 2000, 5);
int _Handle_Motor_Right;
int _Handle_Motor_Left;
int _Handle_Steering_Left;
int _Handle_Steering_Right;
int _Handle_Rear_Center;
_Paras Prs;
_RefWay RefWay;
_SomeFunc Func;
_CarModule CarModule;
float sim_t = 0.;
float T = 0.1;
float ControlV;
int timee = Prs.Timee;
float ControlDelta;
int many = timee / T;
Eigen::VectorXf XReal(many);
Eigen::VectorXf XRef(many);
Eigen::VectorXf YReal(many);
Eigen::VectorXf YRef(many);
Eigen::VectorXf PhiReal(many);
Eigen::VectorXf PhiRef(many);
Eigen::VectorXf VReal(many);
Eigen::VectorXf VRef(many);
Eigen::VectorXf DeltaReal(many);
Eigen::VectorXf DeltaRef(many);
int number;
int main()
{
  Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(Prs.Nx, Prs.Nx);
  Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(Prs.Nx, Prs.Nu);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(Prs.Nx + Prs.Nu, Prs.Nx + Prs.Nu);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(Prs.Nx + Prs.Nu, Prs.Nu);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(Prs.Nx, Prs.Nx + Prs.Nu);
  Eigen::MatrixXd Kesi = Eigen::MatrixXd::Zero(Prs.Nx + Prs.Nu, 1);
  Eigen::MatrixXd StatusReal = Eigen::MatrixXd::Zero(Prs.Nx, 1);
  Eigen::MatrixXd StatusRef = Eigen::MatrixXd::Zero(Prs.Nx, 1);
  Eigen::MatrixXd ControlRef = Eigen::MatrixXd::Zero(Prs.Nu, 1);
  Eigen::MatrixXd ControlReal = Eigen::MatrixXd::Zero(Prs.Nu, 1);
  Eigen::MatrixXd qp_PHI = Eigen::MatrixXd::Zero(Prs.Nx * Prs.Np, Prs.Nx + Prs.Nu);
  Eigen::MatrixXd qp_Qieta = Eigen::MatrixXd::Zero(Prs.Nx * Prs.Np, Prs.Nu * Prs.Nc);
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(Prs.Nx * Prs.Np, 1);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(Prs.Nu * Prs.Nc + 1, Prs.Nu * Prs.Nc + 1);
  Eigen::MatrixXd f = Eigen::MatrixXd::Zero(1, Prs.Nu * Prs.Nc + 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Prs.Nx * Prs.Np, Prs.Nx * Prs.Np);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Prs.Nu * Prs.Nc, Prs.Nu * Prs.Nc);
  Q = Prs.Q * Q;
  R = Prs.R * R;
  while (sim_t <= timee) {
    //Get reference path
    RefWay.RefValue(Prs.Choose, sim_t);
    //Get A_ B_
    A_ << 1, 0, -T * RefWay.Ref_V * sin(RefWay.Ref_Phi),
        0, 1, T * RefWay.Ref_V * cos(RefWay.Ref_Phi),
        0, 0, 1;
    B_ << T * cos(RefWay.Ref_Phi), 0,
        T * sin(RefWay.Ref_Phi), 0,
        T * tan(RefWay.Ref_Delta) / Prs.Cl, T * RefWay.Ref_V / (Prs.Cl * cos(RefWay.Ref_Delta) * cos(RefWay.Ref_Delta));
    //Get A B C
    A << A_, B_,
        Eigen::MatrixXd::Zero(Prs.Nu, Prs.Nx), Eigen::MatrixXd::Identity(Prs.Nu, Prs.Nu);
    std::cout<<"============================================================="<<std::endl;
    std::cout<<A<<std::endl;
    std::cout<<"============================================================="<<std::endl;
    B << B_,
        Eigen::MatrixXd::Identity(Prs.Nu, Prs.Nu);
    std::cout<<"============================================================="<<std::endl;
    std::cout<<B<<std::endl;
    std::cout<<"============================================================="<<std::endl;
    C << Eigen::MatrixXd::Identity(Prs.Nx, Prs.Nx), Eigen::MatrixXd::Zero(Prs.Nx, Prs.Nu);
    std::cout<<"============================================================="<<std::endl;
    std::cout<<C<<std::endl;
    std::cout<<"============================================================="<<std::endl;
    //Get kesi
    ControlRef << RefWay.Ref_V, RefWay.Ref_Delta;
    StatusRef << RefWay.Ref_X, RefWay.Ref_Y, RefWay.Ref_Phi;
    StatusReal << CarModule.Xk, CarModule.Yk, CarModule.Phik;
    Kesi << StatusReal - StatusRef,
        ControlReal - ControlRef;
    //Get phi qieta
    qp_PHI = Func.give_PHI(C, A);
    qp_Qieta = Func.give_Qieta(C, A, B);

    //Get E H f
    E = qp_PHI * Kesi;
    H << qp_Qieta.transpose() * Q * qp_Qieta + R, Eigen::MatrixXd::Zero(Prs.Nu * Prs.Nc, 1),
        Eigen::MatrixXd::Zero(1, Prs.Nu * Prs.Nc), Prs.Row;
    f << E.transpose() * Q * qp_Qieta, 0.;
    H = 0.5 * (H + H.transpose());
    f = f.transpose();
    Eigen::SparseMatrix<double> hessian = H.sparseView();
    Eigen::VectorXd gradient = Func.turn_Vec(f);
    //Get AI lb ub
    Eigen::SparseMatrix<double> LinearMatrix = Func.give_AI();
    Eigen::VectorXd lowerBound = Func.give_lb_ub(Kesi, -1);
    Eigen::VectorXd upperBound = Func.give_lb_ub(Kesi, 1);

    //Get △U
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(Prs.Nu * Prs.Nc + 1); //change
    solver.data()->setNumberOfConstraints(Prs.Nu * Prs.Nc);
    if (!solver.data()->setHessianMatrix(hessian))
      return 1;

    if (!solver.data()->setGradient(gradient))
      return 1;

    if (!solver.data()->setLinearConstraintsMatrix(LinearMatrix))
      return 1;

    if (!solver.data()->setLowerBound(lowerBound))
      return 1;

    if (!solver.data()->setUpperBound(upperBound))
      return 1;

    if (!solver.initSolver())
      return 1;
    Eigen::VectorXd QPSolution;
    for (int j = 0; j <= 100; j++) {
      if (!solver.updateBounds(lowerBound, upperBound))
        return 1;
      if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return 1;
      QPSolution = solver.getSolution();
    }
    ControlV = QPSolution(0);
    ControlDelta = QPSolution(1);
    //Judge bound
    if (ControlV > Prs.DUmax0) {
      ControlV = Prs.DUmax0;
    } else if (ControlV < -Prs.DUmax0) {
      ControlV = -Prs.DUmax0;
    }
    if (ControlDelta > Prs.DUmax1) {
      ControlDelta = Prs.DUmax1;
    } else if (ControlDelta < -Prs.DUmax1) {
      ControlDelta = -Prs.DUmax1;
    }

    //Get real control
    ControlReal(0, 0) = Kesi(3, 0) + ControlV + RefWay.Ref_V;
    ControlReal(1, 0) = Kesi(4, 0) + ControlDelta + RefWay.Ref_Delta;

    //Set bound
    if(ControlReal(0,0) < 0){
      ControlReal(0,0) = 0;
    }
    else if(ControlReal(0,0) > Prs.Ov){
      ControlReal(0,0) = Prs.Ov;
    }
    if (ControlReal(1, 0) < -45 * 3.1415926 / 180) {
      ControlReal(1, 0) = -45 * 3.1415926 / 180;
    } else if (ControlReal(1, 0) > 45 * 3.1415926 / 180) {
      ControlReal(1, 0) = 45 * 3.1415926 / 180;
    }

    CarModule.CarMove(T, Prs.Cl, ControlReal(0, 0), ControlReal(1, 0));

    f = f.transpose();
    XReal(number) = StatusReal(0, 0);
    XRef(number) = StatusRef(0, 0);
    YReal(number) = StatusReal(1, 0);
    YRef(number) = StatusRef(1, 0);
    PhiReal(number) = StatusReal(2, 0);
    PhiRef(number) = StatusRef(2, 0);
    VReal(number) = ControlReal(0, 0);
    VRef(number) = ControlRef(0, 0);
    DeltaReal(number) = ControlReal(1, 0);
    DeltaRef(number) = ControlRef(1, 0);
    std::cout << "======================================================================================================" << std::endl;
    std::cout << "Error:" << StatusReal - StatusRef << std::endl;
    std::cout << "======================================================================================================" << std::endl;
    std::cout << "======================================================================================================" << std::endl;
    std::cout << "XReal:" << CarModule.Xk << std::endl;
    std::cout << "XRef:" << RefWay.Ref_X << std::endl;
    std::cout << "YReal:" << CarModule.Yk << std::endl;
    std::cout << "YRef:" << RefWay.Ref_Y << std::endl;
    std::cout << "PhiReal:" << CarModule.Phik << std::endl;
    std::cout << "PhiRef:" << RefWay.Ref_Phi << std::endl;
    std::cout << "======================================================================================================" << std::endl;
    sim_t = sim_t + T;
    ++number;
  }
  std::cout << "================XReal================" << std::endl;
  std::cout << XReal.transpose() << std::endl;
  std::cout << "================XRef================" << std::endl;
  std::cout << XRef.transpose() << std::endl;
  std::cout << "================YReal================" << std::endl;
  std::cout << YReal.transpose() << std::endl;
  std::cout << "================YRef================" << std::endl;
  std::cout << YRef.transpose() << std::endl;
  std::cout << "================PhiReal================" << std::endl;
  std::cout << PhiReal.transpose() << std::endl;
  std::cout << "================PhiRef================" << std::endl;
  std::cout << PhiRef.transpose() << std::endl;
  std::cout << "================VReal================" << std::endl;
  std::cout << VReal.transpose() << std::endl;
  std::cout << "================VRef================" << std::endl;
  std::cout << VRef.transpose() << std::endl;
  std::cout << "================DeltaReal================" << std::endl;
  std::cout << DeltaReal.transpose() << std::endl;
  std::cout << "================DeltaRef================" << std::endl;
  std::cout << DeltaRef.transpose() << std::endl;
  return 0;
}
