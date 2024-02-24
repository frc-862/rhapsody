// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.CollisionConstants;
import frc.robot.subsystems.CollisionDetector;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;
public CollisionDetector collisionDetector;
public double[] angularVelocityWorldLog = {0d, 0d};
public double[] timeLog = {0d, 0d};
public double[] robotRotationFromMotor = {0d, 0d};
public double[] velocityXChassis = {0d, 0d};
public double[] velocityYChassis = {0d, 0d};
public double[] velocityRotChassis = {0d, 0d};

  public CollisionDetection(Swerve drivetrain, CollisionDetector collisionDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.collisionDetector = collisionDetector;
    addRequirements(collisionDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // store ang vel and time to calc angular acceleration

    angularVelocityWorldLog[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    timeLog[1] = Utils.getCurrentTimeSeconds();
    robotRotationFromMotor[1] = drivetrain.getState().Pose.getRotation().getRadians();
    velocityXChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
    velocityYChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
    velocityRotChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // store ang vel and time to calc angular acceleration

    angularVelocityWorldLog[0] = angularVelocityWorldLog[1];
    timeLog[0] = timeLog[1];
    robotRotationFromMotor[0] = robotRotationFromMotor[1];
    velocityXChassis[0] = velocityXChassis[1];
    velocityYChassis[0] = velocityYChassis[1];
    velocityRotChassis[0] = velocityRotChassis[1];

    angularVelocityWorldLog[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    timeLog[1] = Utils.getCurrentTimeSeconds();
    robotRotationFromMotor[1] = drivetrain.getState().Pose.getRotation().getRadians();
    velocityXChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
    velocityYChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
    velocityRotChassis[1] = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;

    LightningShuffleboard.setDouble("Collision Detection", "total pidgeon acceleration", getPigeonAccelerationXYMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", getPidgeonXYAccelerationDirection().getDegrees());
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAngularAcceleration() * CollisionConstants.DISTANCE_FROM_CENTER_TO_MODULE);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon angular velocity", Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()));
    LightningShuffleboard.setDouble("Collision Detection", "yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", getChassisXYAcceleration());
    LightningShuffleboard.setDouble("Collision Detection", "motor angular acceleration", getChassisRotAcceleration());
    LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
// GET INFO FROM PIGEON

  /**
   * @return acceleration from pigeon in x direction is m/s^2
   */
  public double getPigeonAccelerationX(){  
    // use pythagrean theorum to find total acceleration
    return drivetrain.getPigeon2().getAccelerationX().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorX().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2
  }

  /**
   * @return acceleration from pigeon in y direction is m/s^2
   */
  public double getPigeonAccelerationY(){  
    // use pythagrean theorum to find total acceleration
    return (drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorY().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY); // convert g-force to m/s^2
  }

  /**
   * @return pigeon xy acceleration magnitude
   */
  public double getPigeonAccelerationXYMagnitude(){
    return Math.hypot(getPigeonAccelerationX(), getPigeonAccelerationY());
  }

  /**
   * @return direction of x/y acceleration in radians
   */
  public Rotation2d getPidgeonXYAccelerationDirection(){
    return Rotation2d.fromRadians(Math.atan2(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble(),
    drivetrain.getPigeon2().getAccelerationX().getValueAsDouble()));
  }

  /**
   * @return pigeon angular velocity in radians per second
   */
  public double getPigeonAngularAcceleration(){
    return (angularVelocityWorldLog[1] - angularVelocityWorldLog[0]) / (timeLog[1] - timeLog[0]);
  }

// GET CHASSIS SPEEDS (motor probably)

  /**
   * @return acceleration of chasis in x direction
   */
  public double getChassisXAcceleration(){
    return velocityXChassis[1] - velocityXChassis[0] / timeLog[1] - timeLog[0];
  }

  /**
   * @return acceleration of chasis in y direction
   */
  public double getChassisYAcceleration(){
    return velocityYChassis[1] - velocityYChassis[0] / timeLog[1] - timeLog[0];
  }

  /**
   * @return acceleration of chasis rotations
   */
  public double getChassisRotAcceleration(){
      return velocityRotChassis[1] - velocityRotChassis[0] / timeLog[1] - timeLog[0];
  }

  /**
   * @return magnitude of chasis x and y acceleration
   */
  public double getChassisXYAcceleration(){
    return Math.hypot(getChassisXAcceleration(), getChassisYAcceleration());
  }

  // COMPARE CHASIS AND PIGEON

  public boolean getIfCollided(){
    double differenceX = Math.abs(getPigeonAccelerationX() - getChassisXAcceleration());
    boolean xCollided = differenceX > getPigeonAccelerationX() * CollisionConstants.ACCELERATION_TOLERANCE 
    || differenceX > CollisionConstants.ACCELERATION_TOLERANCE;

    double differenceY = Math.abs(getPigeonAccelerationY() - getChassisYAcceleration());
    boolean yCollided = differenceY > getPigeonAccelerationY() * CollisionConstants.ACCELERATION_TOLERANCE 
    || differenceY > CollisionConstants.ACCELERATION_TOLERANCE;

    double differenceRot = Math.abs(getPigeonAngularAcceleration() - getChassisRotAcceleration());
    boolean rotCollided = differenceRot > getPigeonAngularAcceleration() * CollisionConstants.ACCELERATION_TOLERANCE 
    || differenceX > CollisionConstants.ACCELERATION_TOLERANCE;

    return xCollided || yCollided || rotCollided;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}