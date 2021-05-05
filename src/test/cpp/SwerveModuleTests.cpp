#include <gtest/gtest.h>
#include "tigertronics/SwerveModule.h"
#include <frc/kinematics/SwerveModuleState.h>


class SwerveModuleTests : public testing::Test {
protected:
    int encoderTicksPerRev = 4096;
    double gearing = 1.0;
    units::meter_t wheelRadius = 1_m;
};

TEST_F(SwerveModuleTests, SwerveStateZero) {
    double actualAngle = SwerveModule::ConvertSwerveModuleAngleToTalonTicks(0_deg, encoderTicksPerRev, gearing);
    double actualSpeed = SwerveModule::ConvertSwerveModuleSpeedToTalonTickVel(0_fps, wheelRadius, encoderTicksPerRev, gearing);
    EXPECT_DOUBLE_EQ(0, actualAngle);
    EXPECT_DOUBLE_EQ(0, actualSpeed);
};

TEST_F(SwerveModuleTests, SwerveStateSimple) {
    double actualAngle = SwerveModule::ConvertSwerveModuleAngleToTalonTicks(45_deg, encoderTicksPerRev, gearing);
    double actualSpeed = SwerveModule::ConvertSwerveModuleSpeedToTalonTickVel(wpi::math::pi * 1_mps * 2, wheelRadius, encoderTicksPerRev, gearing);
    EXPECT_DOUBLE_EQ(512, actualAngle);
    EXPECT_DOUBLE_EQ(409.6, actualSpeed);
};

TEST_F(SwerveModuleTests, SwerveOptimizeTest) {
    frc::SwerveModuleState currentState;
    currentState.angle = frc::Rotation2d(1077_deg);
    currentState.speed = 1_mps;

    frc::SwerveModuleState desiredState;
    desiredState.angle = frc::Rotation2d(0_deg);
    desiredState.speed = 1_mps;

    frc::SwerveModuleState frcOptimized = frc::SwerveModuleState::Optimize(desiredState, currentState.angle);
    frc::SwerveModuleState ctreOptimized = SwerveModule::Optimize(desiredState, currentState.angle);

    EXPECT_NE(frcOptimized.angle.Degrees(), ctreOptimized.angle.Degrees());
};