// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/IntakeExtenderSubsystem.h"

#include <frc2/command/RunCommand.h>

#include "Constants.h"

#include <rmb/io/log.h>

IntakeExtenderSubsystem::IntakeExtenderSubsystem()
    : extender(intakeSubsystem::extenderID, intakeSubsystem::extenderPIDConfig,
               intakeSubsystem::extenderConvertion,
               intakeSubsystem::extenderFeedforward,
               {intakeSubsystem::extenderFollower}) {
  extender.resetRefrence(0.0_m);
  SetDefaultCommand(frc2::RunCommand([this]() { retract(); }, {this}));
}

// This method will be called once per scheduler run
void IntakeExtenderSubsystem::Periodic() {
  wpi::outs() << std::to_string(extender.getPosition().to<double>()) << wpi::endl;
}

void IntakeExtenderSubsystem::extend() {
  extender.setPosition(intakeSubsystem::extenderOut);
}

std::unique_ptr<frc2::Command> IntakeExtenderSubsystem::extendCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([this]() { extend(); }, {this}));
}

void IntakeExtenderSubsystem::retract() {
  extender.setPosition(intakeSubsystem::extenderIn);
}

std::unique_ptr<frc2::Command> IntakeExtenderSubsystem::retractCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([this]() { retract(); }, {this}));
}

bool IntakeExtenderSubsystem::isExtended() const {
  // TODO: Needs to be implemented
 return (const_cast<rmb::SparkMaxPositionController<units::meters>*>(&extender))->atPosition(intakeSubsystem::extenderOut);
}