// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testcmds.DriveTest;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class DrivetrainSystemTest extends SystemTestCommandGroup {
  
  public DrivetrainSystemTest(Swerve drivetrain, SwerveRequest brake, double speed) {
    super(
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new TimedCommand(new DriveTest(drivetrain, () -> speed, () -> 0d), 1), // Forward
        new WaitCommand(1),
        new TimedCommand(new DriveTest(drivetrain, () -> -speed, () -> 0d), 1), // Backward
        new WaitCommand(1),
        new TimedCommand(new DriveTest(drivetrain, () -> 0d, () -> speed), 1), // Left
        new WaitCommand(1),
        new TimedCommand(new DriveTest(drivetrain, () -> 0d, () -> -speed), 1), // Right
        new WaitCommand(0.5),
        drivetrain.applyRequest(() -> brake) // Brake
      )
    );
  }

}
