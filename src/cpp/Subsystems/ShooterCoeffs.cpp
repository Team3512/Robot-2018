#include "Subsystems/ShooterCoeffs.h"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<1, 1, 1> MakeShooterPlantCoeffs() {
  Eigen::Matrix<double, 1, 1> A;
  A(0, 0) = 0.9974775192550039;
  Eigen::Matrix<double, 1, 1> Ainv;
  Ainv(0, 0) = 1.0025288597450097;
  Eigen::Matrix<double, 1, 1> B;
  B(0, 0) = 0.6216972081698791;
  Eigen::Matrix<double, 1, 1> C;
  C(0, 0) = 1;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  return frc::StateSpacePlantCoeffs<1, 1, 1>(A, Ainv, B, C, D);
}

frc::StateSpaceControllerCoeffs<1, 1, 1> MakeShooterControllerCoeffs() {
  Eigen::Matrix<double, 1, 1> K;
  K(0, 0) = 0.8623214751211266;
  Eigen::Matrix<double, 1, 1> Kff;
  Kff(0, 0) = 0.6200031001383457;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<1, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<1, 1, 1> MakeShooterObserverCoeffs() {
  Eigen::Matrix<double, 1, 1> L;
  L(0, 0) = 0.9973777913974091;
  return frc::StateSpaceObserverCoeffs<1, 1, 1>(L);
}

frc::StateSpaceLoop<1, 1, 1> MakeShooterLoop() {
  return frc::StateSpaceLoop<1, 1, 1>(MakeShooterPlantCoeffs(),
                                      MakeShooterControllerCoeffs(),
                                      MakeShooterObserverCoeffs());
}
