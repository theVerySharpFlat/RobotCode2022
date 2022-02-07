// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {

  throttle = this->shuffleBoardTab.Add("Throttle Speed", 0.2f).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  smPositionController.resetRefrence(0.0_m);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
smPositionController.setPosition(1_m);
//wpi::outs() << "position: " << std::to_string(smPositionController.getPosition().to<double>()) << "m\n";

  //const auto JSX = this->joystick.getX();
  // if(JSX)
  // {
  //   this->smMotorControllerFL.setVelocity(5_mps);
  // }
  // else
  // {
  //   this->smMotorControllerFL.setVelocity(0_mps);
  // }
  // smPositionController.setPosition(6.28_rad);

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
