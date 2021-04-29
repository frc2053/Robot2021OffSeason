#include <gtest/gtest.h>
#include "tigertronics/Util.h"

constexpr double DELTA = 1e-2;

class UnitConversionTests : public testing::Test {
protected:
    int encoderTicksPerRev = 4096;
};

TEST_F(UnitConversionTests, TicksToDistZero) {
    units::meter_t actual = Util::ConvertEncoderTicksToDistance(0, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_EQ(0, actual.to<double>(), DELTA);
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
    int actual = Util::ConvertDistanceToEncoderTicks(0_m, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_EQ(0, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksSimple) {
    int actual = Util::ConvertDistanceToEncoderTicks(wpi::math::pi * 1_m, encoderTicksPerRev, 1.0, .5_m);
    EXPECT_EQ(encoderTicksPerRev, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksGearing) {
    int actual = Util::ConvertDistanceToEncoderTicks(wpi::math::pi * 1_m, encoderTicksPerRev, 1.0/2.0, .5_m);
    EXPECT_EQ(encoderTicksPerRev / 2, actual);
};

TEST_F(UnitConversionTests, DistanceToTicksComplex) {
    int actual = Util::ConvertDistanceToEncoderTicks(-wpi::math::pi * 1_m, encoderTicksPerRev, 2/1, .5_m);
    EXPECT_EQ(-encoderTicksPerRev * 2, actual);
};