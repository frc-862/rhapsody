// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collision extends SubsystemBase {
  private Swerve drivetrain;
  public double pitch;
  public double roll;

  public Collision(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    // initiallizes pitch and roll (so we don't print null values)
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

    // displays whether the robot is off balance
    LightningShuffleboard.setBoolSupplier("Collision", "offBalance", () -> (Math.abs(pitch) > VisionConstants.COLLISION_DEADZONE || Math.abs(roll) > VisionConstants.COLLISION_DEADZONE));
    LightningShuffleboard.setDoubleSupplier("Collision", "pitch", () -> pitch);
    LightningShuffleboard.setDoubleSupplier("Collision", "roll", () -> roll);

  }

  @Override
  public void periodic() {
    // Updates roll and pitch values
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
  }
}
