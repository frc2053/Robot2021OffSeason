// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"
#include "tigertronics/Util.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveSubsystem::SwerveSubsystem() 
    : frontLeftModule{std::make_unique<SwerveModule>(constants::can_ids::FL_SWERVE_DRIVE_ID, constants::can_ids::FL_SWERVE_ROT_ID, constants::drivetrain_motor_config::FL_SWERVE_TURN_MOTOR_CALIBRATION_VALUE, "FL")},
      frontRightModule{std::make_unique<SwerveModule>(constants::can_ids::FR_SWERVE_DRIVE_ID, constants::can_ids::FR_SWERVE_ROT_ID, constants::drivetrain_motor_config::FR_SWERVE_TURN_MOTOR_CALIBRATION_VALUE, "FR")},
      backLeftModule{std::make_unique<SwerveModule>(constants::can_ids::BL_SWERVE_DRIVE_ID, constants::can_ids::BL_SWERVE_ROT_ID, constants::drivetrain_motor_config::BL_SWERVE_TURN_MOTOR_CALIBRATION_VALUE, "BL")},
      backRightModule{std::make_unique<SwerveModule>(constants::can_ids::BR_SWERVE_DRIVE_ID, constants::can_ids::BR_SWERVE_ROT_ID, constants::drivetrain_motor_config::BR_SWERVE_TURN_MOTOR_CALIBRATION_VALUE, "BR")},
      odometry{kinematics, frc::Rotation2d(0_deg), frc::Pose2d()},
      logger{spdlog::get("Swerve")}
{
    SetName("SwerveSubsystem");
    SetSubsystem("SwerveSubsystem");
    AddChild("Front Left Module", frontLeftModule.get());
    AddChild("Front Right Module", frontRightModule.get());
    AddChild("Back Left Module", backLeftModule.get());
    AddChild("Back Right Module", backRightModule.get());

    gyro.Calibrate();
    gyro.ZeroYaw();

    ResetOdom(frc::Pose2d(), frc::Rotation2d(0_deg));
    frc::SmartDashboard::PutData("Pose Field", &poseField);
    frc::SmartDashboard::PutData("Odom Field", &odomField);
    logger->info("Subsystem constructor done");
}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rotSpeed, bool fieldRelative) {
    auto states = kinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, GetGyroAngle()) :
                        frc::ChassisSpeeds{xSpeed, ySpeed, rotSpeed}
    );

    kinematics.NormalizeWheelSpeeds(&states, constants::physical_constants::DRIVETRAIN_MAX_SPEED);

    auto [fl, fr, bl, br] = states;

    frontLeftModule->SetDesiredState(fl);
    frontRightModule->SetDesiredState(fr);
    backLeftModule->SetDesiredState(bl);
    backRightModule->SetDesiredState(br);
}

void SwerveSubsystem::DriveWithJoystick(double x, double y, double rot, bool fieldRelative) {
    const auto xSpeed = -xSpeedLimiter.Calculate(Util::Deadband(x, 0.05)) * constants::physical_constants::DRIVETRAIN_MAX_SPEED;
    const auto ySpeed = -ySpeedLimiter.Calculate(Util::Deadband(y, 0.05)) * constants::physical_constants::DRIVETRAIN_MAX_SPEED;
    const auto rotSpeed = -rotSpeedLimiter.Calculate(Util::Deadband(rot, 0.05)) * constants::physical_constants::DRIVETRAIN_MAX_ROTATION_SPEED;
    Drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
}

void SwerveSubsystem::Periodic() {
    UpdateOdometry();
}

frc::Rotation2d SwerveSubsystem::GetGyroAngle() {
    return frc::Rotation2d(units::degree_t(-gyro.GetYaw()));
}

void SwerveSubsystem::ResetOdom(const frc::Pose2d& pose, const frc::Rotation2d& rot) {
    frontLeftModule->ResetEncoders();
    frontRightModule->ResetEncoders();
    backLeftModule->ResetEncoders();
    backRightModule->ResetEncoders();
    poseEstimator.ResetPosition(pose, rot);
    odometry.ResetPosition(pose, rot);
}

void SwerveSubsystem::UpdateOdometry() {
    odometry.Update(GetGyroAngle(), frontLeftModule->GetState(), frontRightModule->GetState(), backLeftModule->GetState(), backRightModule->GetState());
    poseEstimator.Update(GetGyroAngle(), frontLeftModule->GetState(), frontRightModule->GetState(), backLeftModule->GetState(), backRightModule->GetState());
    poseField.SetRobotPose(poseEstimator.GetEstimatedPosition());
    odomField.SetRobotPose(odometry.GetPose());
}

void SwerveSubsystem::SimulationPeriodic() {
    frontLeftModule->SimulationPeriodic();
    frontRightModule->SimulationPeriodic();
    backLeftModule->SimulationPeriodic();
    backRightModule->SimulationPeriodic();

    HAL_SimDeviceHandle gyroSimHandle = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
    hal::SimDouble angle = HALSIM_GetSimValueHandle(gyroSimHandle, "Yaw");
    angle.Set(-odometry.GetPose().Rotation().Degrees().to<double>());
}
