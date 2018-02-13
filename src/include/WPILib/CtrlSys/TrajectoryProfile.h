/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "CtrlSys/INode.h"

namespace frc {

class TrajectoryProfile : public INode {
 public:
  TrajectoryProfile(double maxV, double maxA, double timeToMaxA);

  void SetTrajectory(std::unique_ptr<Segment[]>& trajectory);

  void SetGoal(double goal, double currentSource = 0.0) override;

  void SetMaxVelocity(double v);
  double GetMaxVelocity() const;
  void SetMaxAcceleration(double a);
  void SetTimeToMaxA(double timeToMaxA);

 protected:
  State UpdateSetpoint(double currentTime) override;

 private:
  Segment* trajectory;
  double m_acceleration;
  double m_maxVelocity;
  double m_profileMaxVelocity;
  double m_timeToMaxA;

  double m_jerk;
  double m_t2;
  double m_t3;
  double m_t4;
  double m_t5;
  double m_t6;
  double m_t7;

  double m_sign;
};

}  // namespace frc
