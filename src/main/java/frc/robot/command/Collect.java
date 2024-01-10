// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Collect extends Command {

  // Declares collector
  private Collector collector;
  private DoubleSupplier powerSupplier;

  public Collect(Collector collector, DoubleSupplier powerSupplier) {
    // Sets collector from parameter
    this.collector = collector;
    this.powerSupplier = powerSupplier;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    collector.setPower(powerSupplier.getAsDouble());
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
