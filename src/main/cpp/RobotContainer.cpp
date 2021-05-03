// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&swerveSubsystem) {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();

    auto teleopDriveCmd = frc2::RunCommand(
        [this] {
            swerveSubsystem.DriveWithJoystick(
                driverController.GetY(frc::GenericHID::kLeftHand),
                driverController.GetX(frc::GenericHID::kLeftHand),
                driverController.GetX(frc::GenericHID::kRightHand),
                true
            );
        },
        {&swerveSubsystem}
    );
    teleopDriveCmd.SetName("TeleopDriveCommand");

    swerveSubsystem.SetDefaultCommand(teleopDriveCmd);
}

void RobotContainer::ConfigureButtonBindings() {
    // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return &m_autonomousCommand;
}
