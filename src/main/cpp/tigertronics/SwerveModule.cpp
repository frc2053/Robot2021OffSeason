#include "tigertronics/SwerveModule.h"
#include "tigertronics/Util.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveMotorCanId, int turningMotorCanId, int calibrationValue, std::string name) 
    : driveMotor(driveMotorCanId),
      turningMotor(turningMotorCanId),
      turningMotorOffsetTicks(calibrationValue),
      moduleName(name) {

    ConfigureDriveMotor();
    ConfigureTurningMotor();
    ResetEncoders();
}

void SwerveModule::ConfigureTurningMotor() {
    turningMotor.ConfigFactoryDefault();
    turningMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
    turningMotor.SetSensorPhase(false);
    turningMotor.SetInverted(false);
    turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    turningMotor.Config_kF(0, constants::drivetrain_pid::SWERVE_TURNING_KF, 10);
    turningMotor.Config_kP(0, constants::drivetrain_pid::SWERVE_TURNING_KP, 10);
    turningMotor.Config_kI(0, constants::drivetrain_pid::SWERVE_TURNING_KI, 10);
    turningMotor.Config_kD(0, constants::drivetrain_pid::SWERVE_TURNING_KD, 10);
    turningMotor.ConfigAllowableClosedloopError(
        0, 
        Util::ConvertAngleToEncoderTicks(
            constants::drivetrain_motor_config::SWERVE_ALLOWABLE_TURN_ERROR,
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        ),
        10);
    turningMotor.ConfigPeakCurrentLimit(0, 10);
    turningMotor.ConfigContinuousCurrentLimit(constants::drivetrain_motor_config::SWERVE_TURN_CURRENT_LIMIT.to<int>(), 10);
    turningMotor.GetSensorCollection().SyncQuadratureWithPulseWidth(0, 0, true, turningMotorOffsetTicks, 10);
}

void SwerveModule::ConfigureDriveMotor() {
    driveMotor.ConfigFactoryDefault();
    driveMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    driveMotor.SetSensorPhase(false);
    driveMotor.SetInverted(false);
    driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    driveMotor.Config_kF(0, constants::drivetrain_pid::SWERVE_DRIVE_KF, 10);
    driveMotor.Config_kP(0, constants::drivetrain_pid::SWERVE_DRIVE_KP, 10);
    driveMotor.Config_kI(0, constants::drivetrain_pid::SWERVE_DRIVE_KI, 10);
    driveMotor.Config_kD(0, constants::drivetrain_pid::SWERVE_DRIVE_KD, 10);
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {
        Util::ConvertAngularVelocityToLinearVelocity(
            Util::ConvertTicksPer100MsToAngularVelocity (
                driveMotor.GetSelectedSensorVelocity(), 
                constants::encoder_info::CTRE_ENCODER_CPR, 
                constants::physical_constants::SWERVE_DRIVE_MOTOR_GEARING
            ), 
            constants::physical_constants::SWERVE_DRIVE_WHEEL_RADIUS
        ),
        frc::Rotation2d(Util::ConvertTicksToAngle(
            turningMotor.GetSelectedSensorPosition(),
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        ))
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
    const auto state = frc::SwerveModuleState::Optimize(desiredState, GetState().angle);

    frc::SmartDashboard::PutNumber(moduleName + " state angle", state.angle.Degrees().to<double>());

    driveMotor.Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity, 
        Util::ConvertAngularVelocityToTicksPer100Ms(
            Util::ConvertLinearVelocityToAngularVelocity(
                state.speed,
                constants::physical_constants::SWERVE_DRIVE_WHEEL_RADIUS
            ),
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_DRIVE_MOTOR_GEARING
        )
    );
    turningMotor.Set(
        ctre::phoenix::motorcontrol::ControlMode::Position,
        Util::ConvertAngleToEncoderTicks(
            state.angle.Radians(),
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        )
    );
}

void SwerveModule::ResetEncoders() {
    turningMotor.GetSimCollection().SetQuadratureRawPosition(0);
    driveMotor.GetSimCollection().SetQuadratureRawPosition(0);
    turningMotor.SetSelectedSensorPosition(0);
    driveMotor.SetSelectedSensorPosition(0);
}

void SwerveModule::SimulationPeriodic() {

    double turnMotorVoltage = turningMotor.GetMotorOutputVoltage();

    frc::SmartDashboard::PutNumber(moduleName + " voltage", turnMotorVoltage);

    turningMotorSim.SetInputVoltage(units::volt_t{turnMotorVoltage});
    driveMotorSim.SetInputVoltage(units::volt_t{driveMotor.GetMotorOutputVoltage()});
    
    turningMotorSim.Update(20_ms);
    driveMotorSim.Update(20_ms);

    units::degree_t angle = turningMotorSim.GetAngle();
    units::revolutions_per_minute_t vel = turningMotorSim.GetVelocity();

    int turningTicks = Util::ConvertAngleToEncoderTicks(
            angle,
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        );

    int turningVelTicks = Util::ConvertAngularVelocityToTicksPer100Ms(
            vel,
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        );

    frc::SmartDashboard::PutNumber(moduleName + " angle", angle.to<double>());
    frc::SmartDashboard::PutNumber(moduleName + " tick pos", turningTicks);

    frc::SmartDashboard::PutNumber(moduleName + " vel", vel.to<double>());
    frc::SmartDashboard::PutNumber(moduleName + " tick vel", turningVelTicks);

    turningMotor.GetSimCollection().SetQuadratureRawPosition(
        turningTicks
    );
    turningMotor.GetSimCollection().SetQuadratureVelocity(
        turningVelTicks
    );

    driveMotor.GetSimCollection().SetQuadratureVelocity(
        Util::ConvertAngularVelocityToTicksPer100Ms(
            driveMotorSim.GetAngularVelocity(),
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_DRIVE_MOTOR_GEARING
        )
    );
}