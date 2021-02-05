// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <string>

#include <frc/DriverStation.h>

Drivetrain Robot::drivetrain;
Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;
frc::Joystick Robot::appendageStick{kAppendageStickPort};
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

Logger Robot::logger;

template <typename AutoService>
void PostAutonEvents(frc3512::AutonomousChooser& chooser, AutoService& auton) {
    auton.Reset();

    while (frc::DriverStation::GetInstance().IsAutonomousEnabled()) {
        auton.PostEvent(kTimeout);
        chooser.YieldToMain();
    }
}

Robot::Robot() {
    m_autonChooser.AddAutonomous(
        "Autoline", [=] { PostAutonEvents(m_autonChooser, autoLine); });
    m_autonChooser.AddAutonomous("Left Position Switch", [=] {
        PostAutonEvents(m_autonChooser, leftSwitch);
    });
    m_autonChooser.AddAutonomous("Center Position Switch", [=] {
        PostAutonEvents(m_autonChooser, centerSwitch);
    });
    m_autonChooser.AddAutonomous("Right Position Switch", [=] {
        PostAutonEvents(m_autonChooser, rightSwitch);
    });
    m_autonChooser.AddAutonomous("Left Position Priority", [=] {
        PostAutonEvents(m_autonChooser, leftPriority);
    });
    m_autonChooser.AddAutonomous("Right Position Priority", [=] {
        PostAutonEvents(m_autonChooser, rightPriority);
    });
    m_autonChooser.AddAutonomous("Left Position Scale", [=] {
        PostAutonEvents(m_autonChooser, leftScale);
    });
    m_autonChooser.AddAutonomous("Center Position Scale", [=] {
        PostAutonEvents(m_autonChooser, centerScale);
    });
    m_autonChooser.AddAutonomous("Right Position Scale", [=] {
        PostAutonEvents(m_autonChooser, rightScale);
    });
    m_autonChooser.AddAutonomous("Left Position Double", [=] {
        PostAutonEvents(m_autonChooser, leftDouble);
    });
    m_autonChooser.AddAutonomous("Right Position Double", [=] {
        PostAutonEvents(m_autonChooser, rightDouble);
    });
    server.SetSource(camera1);

    // camera1.SetVideoMode(PixelFormat.kYUYV, 320, 240, 30)
    camera1.SetResolution(160, 120);
    camera1.SetFPS(30);

    // camera2.SetResolution(640, 480);
    // camera2.SetFPS(30);

    fileSink.SetVerbosityLevels(LogEvent::VERBOSE_ALL);
    consoleSink.SetVerbosityLevels(LogEvent::VERBOSE_WARN);
    logger.AddLogSink(fileSink);
    logger.AddLogSink(consoleSink);
}

void Robot::DisabledInit() {
    m_autonChooser.EndAutonomous();
    drivetrain.StopClosedLoop();
    drivetrain.ResetGyro();
    drivetrain.ResetEncoders();
    intake.SetMotors(MotorState::kIdle);
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
}

void Robot::AutonomousInit() {
    drivetrain.ResetEncoders();
    drivetrain.ResetGyro();
    elevator.ResetEncoder();
    intake.Deploy();
    climber.LockPawl();

    m_autonChooser.AwaitStartAutonomous();
}

void Robot::TeleopInit() {
    m_autonChooser.EndAutonomous();
    drivetrain.StopClosedLoop();
    elevator.StopClosedLoop();
    intake.Deploy();
    intake.Close();
    climber.LockPawl();
    logger.Log(LogEvent("TELEOP INITIALIZED", LogEvent::VERBOSE_INFO));
}

void Robot::TestInit() { m_autonChooser.EndAutonomous(); }

void Robot::RobotPeriodic() {
    if (!elevator.GetBottomHallEffect()) {
        elevator.ResetEncoder();
    }

    for (int i = 2; i <= 12; i++) {
        if (appendageStick.GetRawButtonPressed(i)) {
            Event event{EventType::kButtonPressed, i};
            Robot::PostEvent(event);
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
        if (appendageStick.GetRawButtonReleased(i)) {
            Event event{EventType::kButtonReleased, i};
            Robot::PostEvent(event);
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
    }
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() {
    m_autonChooser.AwaitRunAutonomous();
    logger.Log(LogEvent(
        "Pos Goal: " + std::to_string(drivetrain.GetPositionGoal()) +
            " Pos: " + std::to_string(drivetrain.GetPosition()) +
            " At Goal?: " + std::to_string(drivetrain.AtPositionGoal()),
        LogEvent::VERBOSE_DEBUG));
    logger.Log(
        LogEvent("Angle Goal: " + std::to_string(drivetrain.GetAngleGoal()) +
                     " Angle: " + std::to_string(drivetrain.GetAngle()) +
                     " At Goal?: " + std::to_string(drivetrain.AtAngleGoal()),
                 LogEvent::VERBOSE_DEBUG));
    logger.Log(
        LogEvent("Elevator Position: " + std::to_string(elevator.GetHeight()),
                 LogEvent::VERBOSE_DEBUG));
    drivetrain.Debug();
}

void Robot::TeleopPeriodic() {
    if (driveStick1.GetRawButton(1)) {
        drivetrain.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        drivetrain.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }
    // drivetrain.PostEvent(EventType::kTimeout);
    elevator.PostEvent(EventType::kTimeout);
    climber.PostEvent(EventType::kTimeout);
}

void Robot::HandleEvent(Event event) {
#if 0
    if (event == Event{kButtonPressed, 11}) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }
#endif
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif  // RUNNING_FRC_TESTS
