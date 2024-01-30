// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class CollectorTest extends Command {

  private Collector collector;
  private double power;

  public CollectorTest(Collector collector, double power) {
    this.collector = collector;
    this.power = power;

    addRequirements(collector);
  }

  @Override
  public void initialize() {
    collector.setPower(power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    collector.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
