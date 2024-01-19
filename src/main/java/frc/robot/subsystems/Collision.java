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
  /** Creates a new Collision. */
  public Collision(Swerve drivetrain) {
    this.drivetrain = drivetrain;

    // creates a new element for the shuffleboard
    LightningShuffleboard.setDoubleSupplier("Swerve", "pitch", () -> drivetrain.getPigeon2().getPitch().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Swerve", "roll", () -> drivetrain.getPigeon2().getRoll().getValueAsDouble());
  }

  public double getPitch() {
    // returns the pitch angle
    return pitchAngle;
  }

  public double getRoll() {
    // returns the roll angle
    return rollAngle;
  }

  @Override
  public void periodic() {
    // updates roll, pitch, and the magnitude of those values
    pitchAngle = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    rollAngle = drivetrain.getPigeon2().getRoll().getValueAsDouble();
  }
}
