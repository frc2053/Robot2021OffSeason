#include "tigertronics/SwerveModule.h"
#include "tigertronics/Util.h"
#include "frc/smartdashboard/SmartDashboard.h"

SwerveModule::SwerveModule(int driveMotorCanId, int turningMotorCanId, int calibrationValue, std::string name) 
    : driveMotor(driveMotorCanId),
      turningMotor(turningMotorCanId),
      turningMotorOffsetTicks(calibrationValue),
      moduleName(name),
      logger{spdlog::get("Swerve")} {

    SetName(name + " Module");
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
    double currentEncoderPos = turningMotor.GetSelectedSensorPosition();

    unwrappedAngle = Util::ConvertTicksToAngle(
        currentEncoderPos,
        constants::encoder_info::CTRE_ENCODER_CPR,
        constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING,
        false
    );

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
            currentEncoderPos,
            constants::encoder_info::CTRE_ENCODER_CPR,
            constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING
        ))
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
    const auto state = frc::SwerveModuleState::Optimize(desiredState, GetState().angle);
    currentSimState = state;

    double driveTicks = ConvertSwerveModuleSpeedToTalonTickVel(state.speed);
    double turnTicks = ConvertSwerveModuleAngleToTalonTicks(PlaceInAppropriate0to360Scope(unwrappedAngle, state.angle.Radians()));

    frc::SmartDashboard::PutNumber(moduleName + " Setpoint Deg", state.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber(moduleName + " Drive Ticks", driveTicks);
    frc::SmartDashboard::PutNumber(moduleName + " Module Angle Ticks", turnTicks);

    driveMotor.Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        driveTicks
    );
    turningMotor.Set(
        ctre::phoenix::motorcontrol::ControlMode::Position,
        turnTicks
    );

    turningMotor.GetSimCollection().SetQuadratureRawPosition(turnTicks);
    driveMotor.GetSimCollection().SetQuadratureVelocity(driveTicks);
}

void SwerveModule::OverrideAngleEncoderValues(double turnEncVal) {
    turningMotor.GetSensorCollection().SetQuadraturePosition(turnEncVal);
    turningMotor.GetSimCollection().SetQuadratureRawPosition(turnEncVal);
}

void SwerveModule::ResetEncoders() {
    turningMotor.GetSimCollection().SetQuadratureRawPosition(0);
    driveMotor.GetSimCollection().SetQuadratureRawPosition(0);
    turningMotor.SetSelectedSensorPosition(0);
    driveMotor.SetSelectedSensorPosition(0);

    
    if(moduleName == "FL") {
        turningMotor.GetSimCollection().SetPulseWidthPosition(4000);
    }
    if(moduleName == "FR") {
        turningMotor.GetSimCollection().SetPulseWidthPosition(1024);
    }
    if(moduleName == "BL") {
        turningMotor.GetSimCollection().SetPulseWidthPosition(2048);
    }
    if(moduleName == "BR") {
        turningMotor.GetSimCollection().SetPulseWidthPosition(3000);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    logger->info("Pulse Width Pos: {} Quad Pos: {}", turningMotor.GetSensorCollection().GetPulseWidthPosition(), turningMotor.GetSelectedSensorPosition());
    turningMotor.GetSensorCollection().SyncQuadratureWithPulseWidth(0, 0, true, 0, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    logger->info("Pulse Width Pos: {} Quad Pos: {}", turningMotor.GetSensorCollection().GetPulseWidthPosition(), turningMotor.GetSelectedSensorPosition());
}

double SwerveModule::ConvertSwerveModuleSpeedToTalonTickVel(units::meters_per_second_t speed) {
    return Util::ConvertAngularVelocityToTicksPer100Ms(
        Util::ConvertLinearVelocityToAngularVelocity(
            speed,
            constants::physical_constants::SWERVE_DRIVE_WHEEL_RADIUS
        ),
        constants::encoder_info::CTRE_ENCODER_CPR,
        constants::physical_constants::SWERVE_DRIVE_MOTOR_GEARING
    );
}

units::radian_t SwerveModule::GetCurrentUnwrappedAngle() {
    return unwrappedAngle;
}

double SwerveModule::ConvertSwerveModuleAngleToTalonTicks(units::radian_t angle) {
    return Util::ConvertAngleToEncoderTicks(
        angle,
        constants::encoder_info::CTRE_ENCODER_CPR,
        constants::physical_constants::SWERVE_TURNING_MOTOR_GEARING,
        false
    );
}

units::radian_t SwerveModule::PlaceInAppropriate0to360Scope(units::radian_t scopeReference, units::radian_t newAngle) {
    units::radian_t lowerBound;
    units::radian_t upperBound;
    units::radian_t lowerOffset = units::math::fmod(scopeReference, 2_rad * wpi::math::pi);
    units::radian_t retVal = newAngle;
    if (lowerOffset >= 0_deg) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + ((2_rad * wpi::math::pi) - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - ((2_rad * wpi::math::pi) + lowerOffset);
    }
    while (retVal < lowerBound) {
        retVal = retVal + (2_rad * wpi::math::pi);
    }
    while (retVal > upperBound) {
        retVal = retVal - (2_rad * wpi::math::pi);
    }
    if (retVal - scopeReference > (1_rad * wpi::math::pi)) {
        retVal = retVal - (2_rad * wpi::math::pi);
    } else if (retVal - scopeReference < -(1_rad * wpi::math::pi)) {
        retVal = retVal + (2_rad * wpi::math::pi);
    }
    return retVal;
}

void SwerveModule::SimulationPeriodic() {

}

void SwerveModule::InitSendable(frc::SendableBuilder& builder) {
    builder.SetSmartDashboardType("SwerveModule");
    builder.SetActuator(true);
    builder.AddDoubleProperty("angle", [=]() { return GetState().angle.Degrees().to<double>(); }, nullptr);
    builder.AddDoubleProperty("speed", [=]() { return GetState().speed.to<double>(); }, nullptr);
}