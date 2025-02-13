// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/turret/TurretSubsystem.h"
#include <frc2/command/ConditionalCommand.h>

#include "shooter/turret/TurretFindCommand.h"
#include "shooter/turret/TurretFollowCommand.h"

TurretSubsystem::TurretSubsystem() {
  positionController.setMaxPosition(turretSubsystemConstants::maxPosition);
  positionController.setMinPosition(turretSubsystemConstants::minPosition);
  positionController.resetRefrence(0.0_tr);
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {}

void TurretSubsystem::spinTo(units::angle::radian_t pos) {
  positionController.setPosition(pos);
}

void TurretSubsystem::spinOffset(units::angle::radian_t offset) {
  positionController.spinOffset(offset);
}

units::angle::radian_t TurretSubsystem::getAngularPosition() const{
  return positionController.getPosition();
}

bool TurretSubsystem::isAtPosition(units::angle::radian_t pos) {
  return positionController.atPosition(pos);
}
