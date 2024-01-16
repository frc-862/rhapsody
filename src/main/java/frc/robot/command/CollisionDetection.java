// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;


public class CollisionDetection extends Command {
  // stating drivetrain variables
  private Swerve drivetrain;
  private double pitchAngle;
  private double rollAngle;
  private double deadZone;
  /** Creates a new CollisionDetection. */
  public CollisionDetection(Swerve drivetrain) {
    // Sets drivetrain from parameter
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initializes roll, pitch, and dead zone
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    deadZone = 2d;

    // creates a new element for the shuffleboard
    LightningShuffleboard.setBool("Swerve", "off balance", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // updates roll and pitch
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    // checks if our roll and pitch are larger than the dead zone
    if(Math.abs(pitchAngle) > deadZone || Math.abs(rollAngle) > deadZone){
      LightningShuffleboard.getBool("Swerve", "off balance", true);
    } else{
      LightningShuffleboard.getBool("Swerve", "off balance", false);
    }
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
