// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CollisionDetector;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.TunerConstants;

import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;
public CollisionDetector collisionDetector;
public double pigeonAcceleration;
public double motorAcceleration;
public boolean collided = false;



  public CollisionDetection(Swerve drivetrain, CollisionDetector collisionDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.collisionDetector = collisionDetector;
    addRequirements(collisionDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LightningShuffleboard.setDoubleSupplier("Collision Detection", "ang vel", () -> drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "motor vel", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "x acc", () -> drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "y acc", () -> drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());

    LightningShuffleboard.setDoubleSupplier("Collision Detection", "pidgeon vel", () -> pigeonAcceleration);
    LightningShuffleboard.setDoubleSupplier("Collision Detection", "motor vel", () -> motorAcceleration);
    LightningShuffleboard.setBoolSupplier("Collision Detection", "collided", () -> collided);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pigeonAcceleration = Math.sqrt(Math.pow(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble() - 1, 2) 
    + Math.pow(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() - 1, 2)) * 9.8;

    motorAcceleration = Math.abs(drivetrain.getModule(0).getDriveMotor().getAcceleration().getValueAsDouble() 
    * TunerConstants.kWheelRadiusInches * 2 * Math.PI);

      collided = Math.abs(pigeonAcceleration - motorAcceleration) > VisionConstants.Collision_Acceleration_Tolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public boolean getIfCollided(){
    return collided;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
