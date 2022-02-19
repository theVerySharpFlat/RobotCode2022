// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "RobotContainer.h"
#include <frc/TimedRobot.h>

#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <Constants.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

private:
  //RobotContainer container;

  rev::CANSparkMax velocityController{
    41, rev::CANSparkMaxLowLevel::MotorType::kBrushless
  };
  rev::CANSparkMax velocityController_two{
    22, rev::CANSparkMaxLowLevel::MotorType::kBrushless
  };

  frc::Joystick joystick {
    0
  };
};
