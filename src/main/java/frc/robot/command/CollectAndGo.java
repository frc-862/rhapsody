// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.Constants.CollectorConstants;

public class CollectAndGo extends Command {

  private final Collector collector;

  /** Creates a new CollectAndGo. 
   * @param collector subsystem
  */
  public CollectAndGo(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    collector.setPower(CollectorConstants.COLLECTOR_GRABANDGO_POWER);
  }

  @Override
  public void execute() {
    collector.setPower(CollectorConstants.COLLECTOR_GRABANDGO_POWER);
  }

  @Override
  public void end(boolean interrupted) {
    collector.stop();
  }

  @Override
  public boolean isFinished() {
    return collector.getEntryBeamBreakState();
  }
}
