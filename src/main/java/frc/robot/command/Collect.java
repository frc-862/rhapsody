// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Collector;

public class Collect extends Command {

  // Declares collector
  private Collector collector;

  public Collect(Collector collector) {
    // Sets collector from parameter
    this.collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    collector.setPower(1d);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    collector.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
