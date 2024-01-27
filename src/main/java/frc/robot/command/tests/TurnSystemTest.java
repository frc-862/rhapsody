// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testCommands.TurnTest;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class TurnSystemTest extends SystemTestCommandGroup {

  public TurnSystemTest(Swerve drivetrain, SwerveRequest brake, double speed) {
    super(
      new SequentialCommandGroup(
        new TimedCommand(new TurnTest(drivetrain, () -> speed), 1),
        new WaitCommand(0.5),
        new TimedCommand(new TurnTest(drivetrain, () -> -speed), 1),
        drivetrain.applyRequest(() -> brake)
      )
    );
  }

}
