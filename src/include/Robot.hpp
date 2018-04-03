// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <pathfinder.h>
#include <sys/stat.h>

#include <array>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>

#include <CameraServer.h>
#include <Joystick.h>
#include <PowerDistributionPanel.h>
#include <TimedRobot.h>
#include <Timer.h>
#include <XboxController.h>
#include <cscore.h>
#include <Compressor.h>

#include "AutonomousModes/AutoAutoLine.hpp"
#include "AutonomousModes/AutoCenterScale.hpp"
#include "AutonomousModes/AutoCenterSwitch.hpp"
#include "AutonomousModes/AutoLeftDouble.hpp"
#include "AutonomousModes/AutoLeftScale.hpp"
#include "AutonomousModes/AutoLeftSwitch.hpp"
#include "AutonomousModes/AutoRightDouble.hpp"
#include "AutonomousModes/AutoRightScale.hpp"
#include "AutonomousModes/AutoRightSwitch.hpp"
#include "Constants.hpp"
#include "DSDisplay/DSDisplay.hpp"
#include "ES/Service.hpp"
#include "LiveGrapher/LiveGrapher.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Elevator.hpp"
#include "Subsystems/Intake.hpp"

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

    static std::string GetFileCreationTime(std::string filePath);

    void DS_PrintOut();

    std::string version;

    /**
     * Uses waypoints to generate a trajectory
     *
     * @return a tuple with the center trajectory, the left trajectory, then the
     * right trajectory
     */
    template <size_t N>
    auto GenerateTrajectory(std::array<Waypoint, N>& waypoints);

    static std::unique_ptr<Segment[]> trajectory;
    static std::unique_ptr<Segment[]> leftTrajectory;
    static std::unique_ptr<Segment[]> rightTrajectory;

    static DriveTrain robotDrive;
    static Intake intake;
    static Elevator elevator;
    static frc::Joystick appendageStick;
    static frc::Joystick driveStick1;
    static frc::Joystick driveStick2;

    // LiveGrapher host
    static LiveGrapher liveGrapher;

private:
    AutoAutoLine autoLine;
    AutoCenterScale centerScale;
    AutoCenterSwitch centerSwitch;
    AutoLeftDouble leftDouble;
    AutoLeftScale leftScale;
    AutoLeftSwitch leftSwitch;
    AutoRightDouble rightDouble;
    AutoRightScale rightScale;
    AutoRightSwitch rightSwitch;

    frc::Compressor compressor;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{kDsPort};

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};
    cs::CvSink camera1Sink;
    // cs::CvSink camera2Sink;

    cs::MjpegServer server{"Server", kMjpegServerPort};
};

#include "Robot.inc"
