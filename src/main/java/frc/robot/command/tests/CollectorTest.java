// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class CollectorTest extends Command {
  Collector collector;
  private double power;
  /** Creates a new CollectorTest. */
  public CollectorTest(Collector collector, double power) {
    this.collector = collector;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
