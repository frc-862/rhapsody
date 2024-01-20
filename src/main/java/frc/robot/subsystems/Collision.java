// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collision extends SubsystemBase {
  private Swerve drivetrain;
  public double pitch;
  public double roll;
  /** Creates a new CollisionDetection. */

  public Collision(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    // initiallizes pitch and roll
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

    // displays whether the robot is off balence
    LightningShuffleboard.setBoolSupplier("Collision", "offBalance", () -> (Math.abs(pitch) > VisionConstants.COLLISION_DEADZONE || Math.abs(roll) > VisionConstants.COLLISION_DEADZONE));
  }

  @Override
  public void periodic() {
    // Updates roll and pitch values
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
  }
}
