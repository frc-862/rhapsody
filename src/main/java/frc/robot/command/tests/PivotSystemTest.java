// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testcmds.PivotTest;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class PivotSystemTest extends SystemTestCommandGroup {

  public PivotSystemTest(Pivot pivot, double speed) {
    super(
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new TimedCommand(new PivotTest(pivot, speed), 0.5), // Pivot up
        new WaitCommand(1),
        new TimedCommand(new PivotTest(pivot, -speed), 0.5) // Pivot down
      )
    );
  }

}
