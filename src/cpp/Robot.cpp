// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>

std::unique_ptr<Segment[]> Robot::trajectory;
std::unique_ptr<Segment[]> Robot::leftTrajectory;
std::unique_ptr<Segment[]> Robot::rightTrajectory;

DriveTrain Robot::robotDrive;
Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;
frc::Joystick Robot::appendageStick{kAppendageStickPort};
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

LiveGrapher Robot::liveGrapher{kLiveGrapherPort};

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {}, [] {});
    dsDisplay.AddAutoMethod(
        "Autoline", std::bind(&AutoAutoLine::Reset, &autoLine),
        std::bind(&AutoAutoLine::PostEvent, &autoLine, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Switch", std::bind(&AutoLeftSwitch::Reset, &leftSwitch),
        std::bind(&AutoLeftSwitch::PostEvent, &leftSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Center Position Switch",
        std::bind(&AutoCenterSwitch::Reset, &centerSwitch),
        std::bind(&AutoCenterSwitch::PostEvent, &centerSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Switch",
        std::bind(&AutoRightSwitch::Reset, &rightSwitch),
        std::bind(&AutoRightSwitch::PostEvent, &rightSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Scale", std::bind(&AutoLeftScale::Reset, &leftScale),
        std::bind(&AutoLeftScale::PostEvent, &leftScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Center Position Scale",
        std::bind(&AutoCenterScale::Reset, &centerScale),
        std::bind(&AutoCenterScale::PostEvent, &centerScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Scale", std::bind(&AutoRightScale::Reset, &rightScale),
        std::bind(&AutoRightScale::PostEvent, &rightScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Double", std::bind(&AutoLeftDouble::Reset, &leftDouble),
        std::bind(&AutoLeftDouble::PostEvent, &leftDouble, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Double",
        std::bind(&AutoRightDouble::Reset, &rightDouble),
        std::bind(&AutoRightDouble::PostEvent, &rightDouble, kTimeout));

    std::array<Waypoint, 3> waypoints;
    waypoints[0] = {-4, -1, d2r(45)};
    waypoints[1] = {-1, 2, 0};
    waypoints[2] = {2, 4, 0};

    //    std::tie(trajectory, leftTrajectory, rightTrajectory) =
    //        GenerateTrajectory(waypoints);

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);

    // camera2.SetResolution(640, 480);
    // camera2.SetFPS(30);

    // camera1Sink.SetSource(camera1);
    // camera2Sink.SetSource(camera2);

    server.SetSource(camera1);
}

std::string Robot::GetFileCreationTime(std::string filePath) {
    struct stat attrib;
    stat(filePath.c_str(), &attrib);
    std::string ret(19);
    strftime(&ret[0], 19, "%y-%m-%d %R", localtime(&(attrib.st_ctime)));
    return ret;
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    robotDrive.ResetGyro();
    robotDrive.ResetEncoders();
    intake.SetMotors(MotorState::kIdle);
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
}

void Robot::AutonomousInit() {
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    elevator.ResetEncoder();
    intake.Deploy();

    dsDisplay.ExecAutonomousInit();
}

void Robot::TeleopInit() {
    robotDrive.StopClosedLoop();
    elevator.StopClosedLoop();
    intake.Deploy();
    intake.Close();
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    DS_PrintOut();

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

void Robot::AutonomousPeriodic() { dsDisplay.ExecAutonomousPeriodic(); }

void Robot::TeleopPeriodic() {
    robotDrive.PostEvent(EventType::kTimeout);
    elevator.PostEvent(EventType::kTimeout);
}

void Robot::HandleEvent(Event event) {
    // Camera Switching
    /* if (event == Event{kButtonPressed, 11}) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    } */
}

void Robot::DS_PrintOut() {
    /*if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData(
            (robotDrive.GetLeftRate() + robotDrive.GetRightRate()) / 2,
            "Average Velocity");
        liveGrapher.GraphData(robotDrive.GetAngularRate(), "Angle Rate");
        static double prevVel = 0.0;
        static double curVel = 0.0;
        curVel = robotDrive.GetAngularRate();
        liveGrapher.GraphData((curVel - prevVel) / .005, "Angle Accel");
        prevVel = curVel;
        liveGrapher.ResetInterval();
        */
    //robotDrive.Debug();
    // std::cout << robotDrive.GetLeftDisplacement() << "Left, Right" <<
    // robotDrive.GetRightDisplacement() << std::endl;
    //std::cout << robotDrive.GetAngle() << std::endl;
    //std::cout << elevator.GetHeight() << std::endl;
    // std::cout << "Version 1.5" << std::endl; // To ensure a
    // successful(butchered) upload
    dsDisplay.AddData("ENCODER_LEFT", robotDrive.GetLeftDisplacement());
    dsDisplay.AddData("ENCODER_RIGHT", robotDrive.GetRightDisplacement());
    dsDisplay.AddData("GYRO_VAL", robotDrive.GetAngle());
    dsDisplay.AddData("PAWL_ENGAGED",
                      climber.IsLowGear());  // todo: add the function
    dsDisplay.AddData("LOW_GEAR",
                      climber.IsLowGear());  // todo: add the function
}

START_ROBOT_CLASS(Robot)
