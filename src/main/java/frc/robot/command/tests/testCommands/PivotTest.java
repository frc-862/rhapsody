// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotTest extends Command {

  private Pivot pivot;
  private double angle;

  public PivotTest(Pivot pivot, double angle) {
    this.pivot = pivot;
    this.angle = angle;

    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setTargetAngle(angle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
