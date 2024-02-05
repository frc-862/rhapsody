// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.tests.testCommands.SingTest;
import frc.robot.subsystems.Swerve;
import frc.thunder.testing.SystemTestCommandGroup;

public class OrchestraSystemTest extends SystemTestCommandGroup {

  public OrchestraSystemTest(Swerve drivetrain, String filepath) {
    super(
      new SequentialCommandGroup(
        new SingTest(drivetrain, filepath))
    );
  }
}