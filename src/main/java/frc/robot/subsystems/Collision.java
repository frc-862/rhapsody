// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Swerve;

public class Collision extends SubsystemBase {
  // states our variables for our subsystem
  private Swerve drivetrain;
  private double pitchAngle;
  private double rollAngle;
  private double magnitude;
  public boolean balanced;
  /** Creates a new Collision. */
  public Collision(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    this.balanced = true;

    // creates a new element for the shuffleboard
    LightningShuffleboard.setDoubleSupplier("Swerve", "pitch", () -> drivetrain.getPigeon2().getPitch().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Swerve", "roll", () -> drivetrain.getPigeon2().getRoll().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Swerve", "magnitude", () -> magnitude);
    LightningShuffleboard.setBoolSupplier("Swerve", "balanced", () -> balanced); // TODO: delete this when we figure what to do when off balanced
  }

  // returns the magnitude of our roll and pitch
  public double getMagnitude() {
    return magnitude;
  }

  @Override
  public void periodic() {
    // updates roll, pitch, and the magnitude of those values
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    magnitude = Math.sqrt((pitchAngle * pitchAngle) + (rollAngle * rollAngle));
  }
}
