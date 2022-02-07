// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <frc/geometry/Translation2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <AHRS.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>


namespace driverStationConstants {
  const int driveStickID = 0;
  const double driveStickDeadzone = 0.2;
  const bool squareDriveStick = true;
}

namespace driveSubsystemConstants {

const frc::SPI::Port gyroPort = frc::SPI::kMXP;

const int frontLeftID  = 1, 
          frontRightID = 2, 
          rearLeftID   = 3,
          rearRightID  = 4;

const frc::Translation2d frontLeftPose  = {-0.381_m,  0.381_m},
                         frontRightPose = { 0.381_m,  0.381_m}, 
                         rearLeftPose   = {-0.381_m, -0.381_m},
                         rearRightPose  = { 0.381_m, -0.381_m};

const rmb::SparkMaxVelocityController<units::meters>::PIDConfig
    motorPIDConfig{
        /* p */ 0.0, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 25_mps, /* minVelocity */ 0_mps,
        /* maxAccel */ 10_mps_sq,
        /* allowedErr */ 0.01_mps,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve};

 const rmb::SimpleMotorFeedforward<units::meters>
    motorFeedforward(rmb::SimpleMotorFeedforward<units::meters>::Ks_t(0.10973),
                     rmb::SimpleMotorFeedforward<units::meters>::Kv_t(3.15920),
                     rmb::SimpleMotorFeedforward<units::meters>::Ka_t(0.30746));

const rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t
    motorConvertion(/* radius */(3_in / 1_rad) * /* gearing */(12/1));

const units::meters_per_second_t maxVelocity(2.5_mps);
const units::radians_per_second_t maxRotVelocity(5000_rpm);
const units::radians_per_second_squared_t maxRotAcceleration(5000_rad_per_s_sq);

const frc2::PIDController xController(0.0, 0.0, 0.0), yController(0.0, 0.0, 0.0);
const frc::ProfiledPIDController<units::radians> thetaController(0.0, 0.0, 0.0, {maxRotVelocity, maxRotAcceleration});
} // namespace driveSubsystemConstants

namespace intakeSubsystem {

const static units::meters_per_second_t maxVelocity(0.0);
const static units::radians_per_second_t maxRotVelocity(0.0);
} // namespace driveSubsystemConstants


namespace positionControllerConstants
{
    const static rmb::SparkMaxPositionController<units::meters>::PIDConfig
    positionCtrlConfig{
        /* p */ 0.000, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 3_mps, /* minVelocity */ 0_mps,
        /* maxAccel */ 10_mps_sq,
        /* allowedErr */ 0.01_m,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve
    };
} // namespace positionControllerConstants

