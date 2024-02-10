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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LightningShuffleboard.setDouble("Collision Detection", "pidgeon acc", getPigeonAcceleration());
    LightningShuffleboard.setDouble("Collision Detection", "motor acc", getMotorAcceleration(0));
    LightningShuffleboard.setBool("Collision Detection", "collided", checkMotorAcceleration(0));
    
    if (getIfCollided()){
      // do stuff
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public double getPigeonAcceleration(){

    return Math.sqrt(Math.pow(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(), 2) 
    + Math.pow(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble(), 2)) * 9.8;
  }

  public double getMotorAcceleration(int moduleNumber){

    return Math.abs(drivetrain.getModule(moduleNumber).getDriveMotor().getAcceleration().getValueAsDouble() 
    * TunerConstants.kWheelRadiusInches * 2 * Math.PI / 39.36);
  }

  public boolean checkMotorAcceleration(int moduleNumber){
    return Math.abs(getPigeonAcceleration() - getMotorAcceleration(moduleNumber))
    > VisionConstants.Collision_Acceleration_Tolerance;
  }

  public boolean getIfCollided(){
    return checkMotorAcceleration(0) || checkMotorAcceleration(1) 
    || checkMotorAcceleration(2) || checkMotorAcceleration(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}