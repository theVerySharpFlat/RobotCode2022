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

const int frontLeftID  = 11, 
          frontRightID = 12, 
          rearLeftID   = 13,
          rearRightID  = 14;

const frc::Translation2d frontLeftPose  = { 0.303_m, -0.299_m},
                         frontRightPose = { 0.303_m,  0.299_m}, 
                         rearLeftPose   = {-0.303_m, -0.299_m},
                         rearRightPose  = {-0.303_m,  0.299_m,};
                         
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

const int extenderID = 21, spinnerID = 23;

const rmb::SparkMaxPositionController<units::meters>::Follower extenderFollower = {22, rev::CANSparkMax::MotorType::kBrushless, true};

const rmb::SparkMaxPositionController<units::meters>::PIDConfig
    extenderPIDConfig{
        /* p */ 0.0001, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 0.2_mps, /* minVelocity */ 0.0_mps,
        /* maxAccel */ 0.5_mps_sq,
        /* allowedErr */ 0.05_m,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve};

 const rmb::SimpleMotorFeedforward<units::meters>
    extenderFeedforward(rmb::SimpleMotorFeedforward<units::meters>::Ks_t(0.26336),
                        rmb::SimpleMotorFeedforward<units::meters>::Kv_t(2.6898),
                        rmb::SimpleMotorFeedforward<units::meters>::Ka_t(0.043975));

const rmb::SparkMaxPositionController<units::meters>::ConversionUnit_t
    extenderConvertion(/* radius */(1.3_in / 2_rad) * /* gearing */(1.0/4));

const units::meter_t extenderOut = 0.24_m;
const units::meter_t extenderIn = 0.0_m;

} // namespace driveSubsystemConstants

namespace storageConstants{
  const int wheelID = 31;


}

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

namespace shooterSubsystemConstants {
  const int flywheelMotorID  = 43;

  const rmb::SparkMaxVelocityController<units::meters>::PIDConfig
    flywheelPIDConfig{
        /* p */ 0.0, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 100_mps, /* minVelocity */ 0_mps,
        /* maxAccel */ 10_mps_sq,
        /* allowedErr */ 0.01_mps,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve
    };

  const rmb::SimpleMotorFeedforward<units::meters>
    flywheelFeedforward(rmb::SimpleMotorFeedforward<units::meters>::Ks_t(0.10973),
                     rmb::SimpleMotorFeedforward<units::meters>::Kv_t(3.15920),
                     rmb::SimpleMotorFeedforward<units::meters>::Ka_t(0.30746));

  const auto flywheelConversion = 
       rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(1_m / 1_rad);

}


namespace hoodSubsystemConstants {
    const int motorID = 42;

    const rmb::SparkMaxPositionController<units::radians>::PIDConfig
            hoodPIDConfig{
            /* p */ 16.0, /* i */ 0.0, /* d */ 4.0, /* f */ 0.0,
            /* iZone */ 0.01, /* iMaxAccumulator */ 0.05,
            /* maxOutput */ 1.0, /* minOutput */ -1.0,

            /* SmartMotion config */
            /* usingSmartMotion */ false,
            /* maxVelocity */ 0_tps, /* minVelocity */ 0.0_tps,
            /* maxAccel */ 0_rad_per_s_sq,
            /* allowedErr */ 1.0_deg,
            /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve
    };

    const rmb::SimpleMotorFeedforward<units::radians>
            hoodFeedforward(rmb::SimpleMotorFeedforward<units::radians>::Ks_t(0.15),
                                rmb::SimpleMotorFeedforward<units::radians>::Kv_t(0.0),
                                rmb::SimpleMotorFeedforward<units::radians>::Ka_t(0.0));

    const auto hoodConversion =
            rmb::SparkMaxVelocityController<units::radians>::ConversionUnit_t(22_rad / 32_rad);
}

namespace turretSubsystemConstants {
  const int motorID = 41;

  const units::radian_t minPosition = -90_deg;
  const units::radian_t maxPosition = 180_deg;

  const rmb::SparkMaxPositionController<units::radians>::PIDConfig
    motorPIDConfig{
        /* p */ 0.0000, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 0.1_tps, /* minVelocity */ 0_tps,
        /* maxAccel */ 10_rad_per_s_sq,
        /* allowedErr */ 10_deg,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve
    };

  const rmb::SimpleMotorFeedforward<units::radians>
    motorFeedforward(rmb::SimpleMotorFeedforward<units::radians>::Ks_t(0.50091),
                     rmb::SimpleMotorFeedforward<units::radians>::Kv_t(0.45084),
                     rmb::SimpleMotorFeedforward<units::radians>::Ka_t(0.081423));

  const auto motorConversion = 
       rmb::SparkMaxPositionController<units::radians>::ConversionUnit_t(1_tr / 53.76_tr);
}

