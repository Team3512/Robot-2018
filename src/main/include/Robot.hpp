// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cscore.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "autonomous/AutoAutoLine.hpp"
#include "autonomous/AutoCenterScale.hpp"
#include "autonomous/AutoCenterSwitch.hpp"
#include "autonomous/AutoLeftDouble.hpp"
#include "autonomous/AutoLeftPriority.hpp"
#include "autonomous/AutoLeftScale.hpp"
#include "autonomous/AutoLeftSwitch.hpp"
#include "autonomous/AutoRightDouble.hpp"
#include "autonomous/AutoRightPriority.hpp"
#include "autonomous/AutoRightScale.hpp"
#include "autonomous/AutoRightSwitch.hpp"
#include "dsdisplay/DSDisplay.hpp"
#include "es/Service.hpp"
#include "logging/LogConsoleSink.hpp"
#include "logging/LogFileSink.hpp"
#include "logging/Logger.hpp"
#include "subsystems/CANTalonGroup.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/Intake.hpp"

class Robot : public frc::TimedRobot, public Service {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

    void HandleEvent(Event event) override;

    static Drivetrain drivetrain;
    static Intake intake;
    static Elevator elevator;
    static Climber climber;
    static frc::Joystick appendageStick;
    static frc::Joystick driveStick1;
    static frc::Joystick driveStick2;

    // logging
    static Logger logger;

private:
    AutoAutoLine autoLine;
    AutoCenterScale centerScale;
    AutoCenterSwitch centerSwitch;
    AutoLeftDouble leftDouble;
    AutoLeftPriority leftPriority;
    AutoLeftScale leftScale;
    AutoLeftSwitch leftSwitch;
    AutoRightDouble rightDouble;
    AutoRightPriority rightPriority;
    AutoRightScale rightScale;
    AutoRightSwitch rightSwitch;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{kDsPort};

    // logging Sinks
    LogFileSink fileSink{"/home/lvuser/Robot.log"};
    LogConsoleSink consoleSink;

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};

    cs::MjpegServer server{"Server", kMjpegServerPort};
};
