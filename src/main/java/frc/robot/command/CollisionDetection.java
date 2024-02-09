// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;

  public CollisionDetection(Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LightningShuffleboard.setDoubleSupplier("Collision Detection", "ang vel", () -> drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "motor vel", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "x acc", () -> drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "y acc", () -> drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
