// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testcmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerTest extends Command {

  private Indexer indexer;
  private double power;

  public IndexerTest(Indexer indexer, double power) {
    this.indexer = indexer;
    this.power = power;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setPower(power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    indexer.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
