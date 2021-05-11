// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace constants {
    namespace physical_constants {
        double constexpr SWERVE_DRIVE_MOTOR_GEARING = 7.11 / 1;
        double constexpr SWERVE_TURNING_MOTOR_GEARING = 1.0 / 1;
        units::meter_t constexpr SWERVE_DRIVE_WHEEL_RADIUS = 3_in;
        units::meter_t constexpr DRIVEBASE_LENGTH = 22_in;
        units::meter_t constexpr DRIVEBASE_WIDTH = 22_in;
        units::meters_per_second_t constexpr DRIVETRAIN_MAX_SPEED = 20_fps;
        units::radians_per_second_t constexpr DRIVETRAIN_MAX_ROTATION_SPEED = 180_deg_per_s;
    }

    namespace controller_info {
        int constexpr DRIVER_PORT = 0;
        int constexpr OPERATOR_PORT = 1;
        auto constexpr JOYSTICK_RATE_LIMITER = 200 / 1_s;
    }

    namespace drivetrain_characterization {
        auto constexpr SWERVE_DRIVE_MOTOR_VELOCITY_GAIN = 1.98_V / 1_mps;
        auto constexpr SWERVE_DRIVE_MOTOR_ACCEL_GAIN = 0.1_V / 1_mps_sq;
    }

    namespace drivetrain_pid {
        double constexpr SWERVE_DRIVE_KF = .01;
        double constexpr SWERVE_DRIVE_KP = 1;
        double constexpr SWERVE_DRIVE_KI = 0;
        double constexpr SWERVE_DRIVE_KD = 0;
        double constexpr SWERVE_TURNING_KF = 0;
        double constexpr SWERVE_TURNING_KP = 15;
        double constexpr SWERVE_TURNING_KI = 0;
        double constexpr SWERVE_TURNING_KD = 120;
    }

    namespace encoder_info {
        int constexpr CTRE_ENCODER_CPR = 4096;
    }

    namespace can_ids {
        int constexpr FL_SWERVE_DRIVE_ID = 1;
        int constexpr FR_SWERVE_DRIVE_ID = 2;
        int constexpr BL_SWERVE_DRIVE_ID = 3;
        int constexpr BR_SWERVE_DRIVE_ID = 4;
        int constexpr FL_SWERVE_ROT_ID = 5;
        int constexpr FR_SWERVE_ROT_ID = 6;
        int constexpr BL_SWERVE_ROT_ID = 7;
        int constexpr BR_SWERVE_ROT_ID = 8;
    }

    namespace drivetrain_motor_config {
        units::degree_t constexpr SWERVE_ALLOWABLE_TURN_ERROR = 2_deg;
        units::ampere_t constexpr SWERVE_TURN_CURRENT_LIMIT = 10_A;
        int constexpr FL_SWERVE_TURN_MOTOR_CALIBRATION_VALUE = 4000;
        int constexpr FR_SWERVE_TURN_MOTOR_CALIBRATION_VALUE = 1024;
        int constexpr BL_SWERVE_TURN_MOTOR_CALIBRATION_VALUE = 2048;
        int constexpr BR_SWERVE_TURN_MOTOR_CALIBRATION_VALUE = 3000;
    }
}