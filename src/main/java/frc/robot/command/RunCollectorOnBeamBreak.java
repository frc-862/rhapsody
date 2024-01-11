// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class RunCollectorOnBeamBreak extends Command {

  /*
   * When the beam is interfered with by a note, the collector will run at full power forward.
   * When the beam is not interfered with by a note, the collector will stop.
   */

  // Declare subsystems needed by command
  private Collector collector;

  public RunCollectorOnBeamBreak(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!collector.getEntryBeamBreakState()) {
        collector.setPower(1d);
    } else {
        collector.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
