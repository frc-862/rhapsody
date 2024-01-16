// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Collision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;


public class CollisionDetection extends Command {
  // stating drivetrain variables
  private Swerve drivetrain;
  public Collision collision;
  private double pitchAngle;
  private double rollAngle;
  private double deadZone;
  private boolean offBalance;
  /** Creates a new CollisionDetection. */
  public CollisionDetection(Swerve drivetrain, Collision collision) {
    // Sets drivetrain from parameter
    this.drivetrain = drivetrain;
    this.collision = collision;
    this.offBalance = false;

    addRequirements(collision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initializes roll, pitch, and dead zone
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    deadZone = 2d; // dead zone for how far our robot can tip in degrees

    // creates a new element for the shuffleboard
    LightningShuffleboard.setBool("Swerve", "off balance", offBalance);
    LightningShuffleboard.setDouble("Swerve", "pitch", pitchAngle);
    LightningShuffleboard.setDouble("Swerve", "roll", rollAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // updates roll and pitch
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    // checks if our roll or pitch are larger than the dead zone
    if(Math.abs(pitchAngle) > deadZone || Math.abs(rollAngle) > deadZone){
      offBalance = true;
    } else{
      offBalance = false;
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
