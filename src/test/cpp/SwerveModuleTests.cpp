#include <gtest/gtest.h>
#include "tigertronics/SwerveModule.h"
#include <frc/kinematics/SwerveModuleState.h>


class SwerveModuleTests : public testing::Test {
protected:
    SwerveModule testModule{1, 2, 0, "TEST"};
};

TEST_F(SwerveModuleTests, SwerveStateZero) {
    double actualAngle = SwerveModule::ConvertSwerveModuleAngleToTalonTicks(0_deg);
    double actualSpeed = SwerveModule::ConvertSwerveModuleSpeedToTalonTickVel(0_fps);
    EXPECT_DOUBLE_EQ(0, actualAngle);
    EXPECT_DOUBLE_EQ(0, actualSpeed);
};

TEST_F(SwerveModuleTests, SwerveStateSimple) {
    double actualAngle = SwerveModule::ConvertSwerveModuleAngleToTalonTicks(45_deg);
    double actualSpeed = SwerveModule::ConvertSwerveModuleSpeedToTalonTickVel(0_mps);
    EXPECT_DOUBLE_EQ(512, actualAngle);
    EXPECT_DOUBLE_EQ(0, actualSpeed);
};

TEST_F(SwerveModuleTests, SwerveModuleTickSetpoint) {
    testModule.ResetEncoders();
    testModule.OverrideAngleEncoderValues(5120);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto state = testModule.GetState();
    EXPECT_DOUBLE_EQ(state.angle.Degrees().to<double>(), 90);
    EXPECT_DOUBLE_EQ(testModule.GetCurrentUnwrappedAngle().convert<units::degrees>().to<double>(), 450);
}