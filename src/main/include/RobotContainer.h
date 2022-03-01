// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "driverstation/JoystickSubsystem.h"
#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/TeleopDriveCommand.h"
#include "driverstation/ShuffleBoardSubsystem.h"
#include "storage/StorageSubsystem.h"

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
private:

  void ConfigureButtonBindings();
  // ShooterSubsystem  shooterSubsystem;
  // ClimberSubsystem  climberSubsystem;
  DriveSubsystem    driveSubsystem;
  IntakeExtenderSubsystem intakeExtenderSubsystem;
  IntakeSpinnerSubsystem intakeSpinnerSubsystem{intakeExtenderSubsystem};
  StorageSubsystem storageSubsystem;
  JoystickSubsystem joystickSubsystem;
  TeleopDriveCommand teleopDriveCommand{ driveSubsystem, joystickSubsystem };
  // ShuffleBoardSubsystem  shuffleBoard{ shooterSubsystem, joystickSubsystem, climberSubsystem, driveSubsystem, intakeExtenderSubsystem, intakeSpinnerSubsystem };
};
