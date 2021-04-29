#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <wpi/math>
#include <frc/MathUtil.h>
#include <iostream>

class Util {
public:
    static units::meter_t ConvertEncoderTicksToDistance(double ticks, int encoderResolution, double gearing, units::meter_t wheelRadius) {
        return units::meter_t((ticks / (encoderResolution * gearing)) * (2 * wpi::math::pi * wheelRadius));
    };
    static units::radians_per_second_t ConvertTicksPer100MsToAngularVelocity(double ticksPer100Ms, int encoderResolution, double gearing) {
        return units::radians_per_second_t(ConvertTicksToAngle(ticksPer100Ms, encoderResolution, gearing, false).to<double>() * 10);
    };
    static units::radian_t ConvertTicksToAngle(double ticks, int encoderResolution, double gearing, bool wrap=true) {
        units::radian_t retVal = units::radian_t((ticks / (encoderResolution * gearing)) * (2 * wpi::math::pi));
        return wrap ? frc::AngleModulus(retVal) : retVal;
    };
    static double ConvertDistanceToEncoderTicks(units::meter_t distance, int encoderResolution, double gearing, units::meter_t wheelRadius) {
        return distance * (encoderResolution * gearing) / (wpi::math::pi * 2 * wheelRadius);
    };
    static double ConvertAngularVelocityToTicksPer100Ms(units::radians_per_second_t velocity, int encoderResolution, double gearing) {
        return ConvertAngleToEncoderTicks(units::radian_t(velocity.to<double>()), encoderResolution, gearing, false) / 10.0;
    };
    static double ConvertAngleToEncoderTicks(units::radian_t angle, int encoderResolution, double gearing, bool wrap=true) {
        if(wrap) {
            angle = frc::AngleModulus(angle);
        }
        return angle.to<double>() * (encoderResolution * gearing) / (wpi::math::pi * 2);
    };
    static units::radians_per_second_t ConvertLinearVelocityToAngularVelocity(units::meters_per_second_t linearVelocity, units::meter_t radius) {
        return units::radians_per_second_t(linearVelocity.to<double>() / radius.to<double>());
    };
    static units::meters_per_second_t ConvertAngularVelocityToLinearVelocity(units::radians_per_second_t angularVelocity, units::meter_t radius) {
        return units::meters_per_second_t(angularVelocity.to<double>() * radius.to<double>());
    };
    static double Deadband(double input, double deadband) {
        if (std::abs(input) > deadband) {
            if (input > 0.0) {
                return (input - deadband) / (1.0 - deadband);
            } else {
                return (input + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
private:
};