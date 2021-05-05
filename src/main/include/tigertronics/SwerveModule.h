#pragma once

#include <frc/kinematics/SwerveModuleState.h>
//#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include "Constants.h"
#include <string>
#include <spdlog/spdlog.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>

class SwerveModule : public frc::Sendable,
                     public frc::SendableHelper<SwerveModule> {
public:
    SwerveModule(int driveMotorCanId, int turningMotorCanId, int calibrationValue, std::string name);
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState& desiredState);
    void SimulationPeriodic();
    void ResetEncoders();
    static double ConvertSwerveModuleSpeedToTalonTickVel(units::meters_per_second_t speed, units::meter_t wheelRadius, int encoderCPR, double gearing);
    static double ConvertSwerveModuleAngleToTalonTicks(units::radian_t angle, int encoderCPR, double gearing);
    void InitSendable(frc::SendableBuilder& builder) override;
    void OverrideAngleEncoderValues(double turnEncVal);
private:
    void ConfigureTurningMotor();
    void ConfigureDriveMotor();
    static units::radian_t PlaceInAppropriate0to360Scope(units::radian_t scopeReference, units::radian_t newAngle);

    //TalonFX does not support simulation vars yet
    //ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX driveMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX turningMotor;
    int turningMotorOffsetTicks;
    std::string moduleName;
    units::radian_t unwrappedAngle;

    frc::sim::FlywheelSim driveMotorSim {
        frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
            constants::drivetrain_characterization::SWERVE_DRIVE_MOTOR_VELOCITY_GAIN,
            constants::drivetrain_characterization::SWERVE_DRIVE_MOTOR_ACCEL_GAIN
        ),
        frc::DCMotor::Falcon500(1),
        constants::physical_constants::SWERVE_DRIVE_MOTOR_GEARING
    };

    frc::sim::SingleJointedArmSim turningMotorSim {
        frc::DCMotor::Bag(1),
        constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING,
        .1_kg_sq_m,
        1_in,
        -std::numeric_limits<double>::max() * 1_rad,
        std::numeric_limits<double>::max() * 1_rad,
        .5_kg,
        false
    };

    std::shared_ptr<spdlog::logger> logger;
};