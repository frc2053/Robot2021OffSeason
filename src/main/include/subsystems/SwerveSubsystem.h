// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/SlewRateLimiter.h>
#include "tigertronics/SwerveModule.h"
#include <spdlog/spdlog.h>

class SwerveSubsystem : public frc2::SubsystemBase {
public:
    SwerveSubsystem();
    void Periodic() override;
    void SimulationPeriodic() override;
    void UpdateOdometry();
    void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rotSpeed, bool fieldRelative);
    void DriveWithJoystick(double x, double y, double rot, bool fieldRelative);
    void ResetOdom(const frc::Pose2d& pose, const frc::Rotation2d& rot);
    frc::Rotation2d GetGyroAngle();
private:
    units::meter_t halfWidth = constants::physical_constants::DRIVEBASE_WIDTH / 2;
    units::meter_t halfLength = constants::physical_constants::DRIVEBASE_LENGTH / 2;
    frc::Translation2d frontLeftModuleLocation{halfLength, halfWidth};
    frc::Translation2d frontRightModuleLocation{halfLength, -halfWidth};
    frc::Translation2d backLeftModuleLocation{-halfLength, halfWidth};
    frc::Translation2d backRightModuleLocation{-halfLength, -halfWidth};

    std::unique_ptr<SwerveModule> frontLeftModule;
    std::unique_ptr<SwerveModule> frontRightModule;
    std::unique_ptr<SwerveModule> backLeftModule;
    std::unique_ptr<SwerveModule> backRightModule;

    frc::SwerveDriveKinematics<4> kinematics{
        frontLeftModuleLocation,
        frontRightModuleLocation,
        backLeftModuleLocation,
        backRightModuleLocation
    };

    frc::SwerveDriveOdometry<4> odometry;

    frc::SwerveDrivePoseEstimator<4> poseEstimator{
        frc::Rotation2d(),
        frc::Pose2d(),
        kinematics,
        {0.1, 0.1, 0.1},
        {0.05},
        {0.1, 0.1, 0.1}
    };

    AHRS gyro{frc::SPI::kMXP};

    frc::Field2d poseField;
    frc::Field2d odomField;

    frc::SlewRateLimiter<units::scalar> xSpeedLimiter{constants::controller_info::JOYSTICK_RATE_LIMITER};
    frc::SlewRateLimiter<units::scalar> ySpeedLimiter{constants::controller_info::JOYSTICK_RATE_LIMITER};
    frc::SlewRateLimiter<units::scalar> rotSpeedLimiter{constants::controller_info::JOYSTICK_RATE_LIMITER};

    std::shared_ptr<spdlog::logger> logger;
};
