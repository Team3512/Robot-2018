#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State { kInit, kForward, kDrop, kIdle };

static State state;

void Robot::AutoTimedSwitchInit() { state = State::kInit; }

void Robot::AutoTimedSwitchPeriodic() {
  static std::string platePosition;
  platePosition = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  switch (state) {
    case State::kInit:
      robotDrive.Drive(-0.15, 0, false);
      state = State::kForward;
      break;
    case State::kForward:
      if (autoTimer.Get() > 4.0) {
        robotDrive.Drive(0.0, 0, false);
        if (platePosition[kFriendlySwitch] = 'L') {
          intake.AutoOuttake();
        }
        state = State::kIdle;
      }
      break;
    case State::kIdle:
      break;
  }
}

