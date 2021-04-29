#include <gtest/gtest.h>
#include "tigertronics/Util.h"

class UnitConversionTests : public testing::Test {
protected:
    int encoderTicksPerRev = 4096;
};

TEST_F(UnitConversionTests, TicksToDistZero) {
    units::meter_t actual = Util::ConvertEncoderTicksToDistance(0, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

//If the wheel has rotated one time, we have traveled its circumference
//assuming gearing is 1-1
TEST_F(UnitConversionTests, TicksToDistFullRev) {
    units::meter_t actual = Util::ConvertEncoderTicksToDistance(encoderTicksPerRev, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_DOUBLE_EQ(wpi::math::pi, actual.to<double>());
};

//Tests gear ratios
//Gearing is represented as total ratio
//In the example below, the input shaft rotates 2 times for every 1 revolution of the output shaft
TEST_F(UnitConversionTests, TicksToDistGearing) {
    units::meter_t actual = Util::ConvertEncoderTicksToDistance(encoderTicksPerRev, encoderTicksPerRev, 2.0/1.0, .5_m);
    EXPECT_DOUBLE_EQ(wpi::math::pi / 2, actual.to<double>());
};

//We have a 1 meter radius wheel, but only move half an encoder rev
TEST_F(UnitConversionTests, TicksToDistWheelDiam) {
    units::meter_t actual = Util::ConvertEncoderTicksToDistance(encoderTicksPerRev / 2, encoderTicksPerRev, 1.0, 1_m);
    EXPECT_DOUBLE_EQ(wpi::math::pi, actual.to<double>());
};

TEST_F(UnitConversionTests, TicksToAngleZero) {
    units::radian_t actual = Util::ConvertTicksToAngle(0, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

TEST_F(UnitConversionTests, TicksToAngleSimple) {
    units::radian_t actual = Util::ConvertTicksToAngle(encoderTicksPerRev / 2, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(wpi::math::pi, actual.to<double>());
};

TEST_F(UnitConversionTests, TicksToAngleWrapAround) {
    units::radian_t actual = Util::ConvertTicksToAngle(encoderTicksPerRev + encoderTicksPerRev / 2, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(wpi::math::pi, actual.to<double>());
};

TEST_F(UnitConversionTests, TicksToAngle360) {
    units::radian_t actual = Util::ConvertTicksToAngle(encoderTicksPerRev, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

TEST_F(UnitConversionTests, TicksToAngleNegative) {
    units::radian_t actual = Util::ConvertTicksToAngle(-encoderTicksPerRev + encoderTicksPerRev / 2 , encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(wpi::math::pi, actual.to<double>());
};

TEST_F(UnitConversionTests, TickSpeedToAngleSpeedZero) {
    units::radians_per_second_t actual = Util::ConvertTicksPer100MsToAngularVelocity(0, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

//multiply by ten because ticks per 100ms * 10 = ticks per second
TEST_F(UnitConversionTests, TickSpeedToAngleSpeedSimple) {
    units::radians_per_second_t actual = Util::ConvertTicksPer100MsToAngularVelocity(encoderTicksPerRev, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(wpi::math::pi * 2 * 10, actual.to<double>());
};

//multiply by ten because ticks per 100ms * 10 = ticks per second
TEST_F(UnitConversionTests, TickSpeedToAngleSpeedGearing) {
    units::radians_per_second_t actual = Util::ConvertTicksPer100MsToAngularVelocity(encoderTicksPerRev, encoderTicksPerRev, 2.0);
    EXPECT_DOUBLE_EQ((wpi::math::pi * 2 * 10) / 2, actual.to<double>());
};

//multiply by ten because ticks per 100ms * 10 = ticks per second
TEST_F(UnitConversionTests, TickSpeedToAngleSpeedNegative) {
    units::radians_per_second_t actual = Util::ConvertTicksPer100MsToAngularVelocity(-encoderTicksPerRev, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(-(wpi::math::pi * 2 * 10), actual.to<double>());
};

TEST_F(UnitConversionTests, DistanceToTicksZero) {
    double actual = Util::ConvertDistanceToEncoderTicks(0_m, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_DOUBLE_EQ(0, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksSimple) {
    double actual = Util::ConvertDistanceToEncoderTicks(wpi::math::pi * 1_m, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_DOUBLE_EQ(encoderTicksPerRev, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksGearing) {
    double actual = Util::ConvertDistanceToEncoderTicks(wpi::math::pi * 1_m, encoderTicksPerRev, 1.0/2.0, .5_m);
    EXPECT_DOUBLE_EQ(encoderTicksPerRev / 2, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksComplex) {
    double actual = Util::ConvertDistanceToEncoderTicks(-wpi::math::pi * 1_m, encoderTicksPerRev, 2/1, .5_m);
    EXPECT_DOUBLE_EQ(-encoderTicksPerRev * 2, actual);
};

TEST_F(UnitConversionTests, VelocityToTicksZero) {
    double actual = Util::ConvertAngularVelocityToTicksPer100Ms(0_rad_per_s, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(0, actual);
};

//divide by ten because ticks per 1s / 10 = 100ms
TEST_F(UnitConversionTests, VelocityToTicksSimple) {
    double actual = Util::ConvertAngularVelocityToTicksPer100Ms(2 * wpi::math::pi * 1_rad_per_s, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(encoderTicksPerRev / 10.0, actual);
};

//divide by ten because ticks per 1s / 10 = 100ms
TEST_F(UnitConversionTests, VelocityToTicksGear) {
    double actual = Util::ConvertAngularVelocityToTicksPer100Ms(2 * wpi::math::pi * 1_rad_per_s, encoderTicksPerRev, 10);
    EXPECT_DOUBLE_EQ(encoderTicksPerRev, actual);
};

TEST_F(UnitConversionTests, VelocityToTicksComplex) {
    double actual = Util::ConvertAngularVelocityToTicksPer100Ms(-2 * wpi::math::pi * 1_rad_per_s, encoderTicksPerRev, 1.0/2.0);
    EXPECT_DOUBLE_EQ((-encoderTicksPerRev / 2.0) / 10.0, actual);
};

TEST_F(UnitConversionTests, AngleToTicksZero) {
    double actual = Util::ConvertAngleToEncoderTicks(0_rad, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(0, actual);
};

TEST_F(UnitConversionTests, AngleToTicksSimple) {
    double actual = Util::ConvertAngleToEncoderTicks(wpi::math::pi * 1_rad, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(encoderTicksPerRev / 2, actual);
};

TEST_F(UnitConversionTests, AngleToTicksNegative) {
    double actual = Util::ConvertAngleToEncoderTicks((-wpi::math::pi / 2) * 1_rad, encoderTicksPerRev, 1.0);
    EXPECT_DOUBLE_EQ(-encoderTicksPerRev / 4, actual);
};

TEST_F(UnitConversionTests, LinearToAngularVelZero) {
    units::radians_per_second_t actual = Util::ConvertLinearVelocityToAngularVelocity(0_mps, 1_m);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

TEST_F(UnitConversionTests, LinearToAngularVelSimple) {
    units::radians_per_second_t actual = Util::ConvertLinearVelocityToAngularVelocity(5_mps, 2_m);
    EXPECT_DOUBLE_EQ(2.5, actual.to<double>());
};

TEST_F(UnitConversionTests, LinearToAngularVelNegative) {
    units::radians_per_second_t actual = Util::ConvertLinearVelocityToAngularVelocity(-10_mps, 1_m);
    EXPECT_DOUBLE_EQ(-10, actual.to<double>());
};

TEST_F(UnitConversionTests, AngularToLinearVelZero) {
    units::meters_per_second_t actual = Util::ConvertAngularVelocityToLinearVelocity(0_rad_per_s, 1_m);
    EXPECT_DOUBLE_EQ(0, actual.to<double>());
};

TEST_F(UnitConversionTests, AngularToLinearVelSimple) {
    units::meters_per_second_t actual = Util::ConvertAngularVelocityToLinearVelocity(5_rad_per_s, 2_m);
    EXPECT_DOUBLE_EQ(10, actual.to<double>());
};

TEST_F(UnitConversionTests, AngularToLinearVelNegative) {
    units::meters_per_second_t actual = Util::ConvertAngularVelocityToLinearVelocity(-10_rad_per_s, 1_m);
    EXPECT_DOUBLE_EQ(-10, actual.to<double>());
};