// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TipDetection extends Command {
  /** Creates a new TipDetection. */
  private Swerve drivetrain;
  public double pitch;
  public double roll;


  public TipDetection(Swerve drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LightningShuffleboard.setBoolSupplier("Tip Detection", "Tipped",
    () -> (Math.abs(pitch) > VisionConstants.COLLISION_DEADZONE || Math.abs(roll) > VisionConstants.COLLISION_DEADZONE));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

  }

  /**
   * @return whether the robot is tipped
   */
  public boolean isTipped() {
    return (Math.abs(pitch) > VisionConstants.COLLISION_DEADZONE || Math.abs(roll) > VisionConstants.COLLISION_DEADZONE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
