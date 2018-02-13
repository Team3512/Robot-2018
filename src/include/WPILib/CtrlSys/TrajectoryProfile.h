/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <mutex>

#include <pathfinder.h>

#include "CtrlSys/INode.h"

namespace frc {

class TrajectoryProfile : public INode {
 public:
  TrajectoryProfile();

  void SetTrajectory(std::unique_ptr<Segment[]>& trajectory);

 private:
  Segment* trajectory;

  mutable std::mutex m_mutex;
};

}  // namespace frc
