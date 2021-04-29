#include <gtest/gtest.h>
#include "tigertronics/SwerveModule.h"


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