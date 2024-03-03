// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class CollectedNote extends Command {
  private boolean collected_note = false;
  final private double timeout;
  final private Indexer indexer;
  private Timer timer;

  /** Creates a new CollectedNote. */
  public CollectedNote(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeout = 0.5;
    this.indexer = indexer;
  }

  /** Creates a new CollectedNote. */
  public CollectedNote(double timeout, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeout = timeout;
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.collected_note = false;
    this.timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collected_note = collected_note || indexer.hasNote();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !collected_note && timer.hasElapsed(timeout);
  }
}
