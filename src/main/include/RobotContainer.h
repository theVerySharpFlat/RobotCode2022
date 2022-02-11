// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "driverstation/JoystickSubsystem.h"
#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/TeleopDriveCommand.h"

#include <rmb/io/log.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  TeleopDriveCommand &getTeleopDriveCommand() { return teleopDriveCommand; }

  std::unique_ptr<frc2::Command>
  getBasicAutoCommand() {
    wpi::outs() << "Auto COMMAND initialized!" << wpi::endl;
      // Set up config for trajectory
    frc::TrajectoryConfig testTrajectoryConfig(0.5_mps, 0.5_mps_sq);

    testTrajectoryConfig.SetKinematics(driveSubsystemConstants::kinematics);

    frc::Trajectory testTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      {frc::Pose2d(), frc::Pose2d(frc::Translation2d(0.0_m, 1_m),
      frc::Rotation2d())},
      testTrajectoryConfig);
    
    return driveSubsystem.generateTrajectoryCommand(testTrajectory);
  }

private:
  void ConfigureButtonBindings();
  DriveSubsystem driveSubsystem;
  JoystickSubsystem joystickSubsystem;

  TeleopDriveCommand teleopDriveCommand{driveSubsystem, joystickSubsystem};
};
