// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CollisionDetector;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;

import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;
public CollisionDetector collisionDetector;
SwerveRequest.SwerveDriveBrake brake;
public double[] angularVelocityWorldLog;
public double[] timeLog;
public int i = 0;

  public CollisionDetection(Swerve drivetrain, CollisionDetector collisionDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.collisionDetector = collisionDetector;
    addRequirements(collisionDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    brake = new SwerveRequest.SwerveDriveBrake();
    angularVelocityWorldLog[i] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()); //TODO: test
    timeLog[i] = Utils.getCurrentTimeSeconds();
    i++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LightningShuffleboard.setDouble("Collision Detection", "pidgeon acceleration", getTotalPigeonAccelerationMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration", getMotorAccelerationMagnitude(0));
    LightningShuffleboard.setBool("Collision Detection", "motor zero collided", checkMotorAcceleration(0));
    LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided());
    
    if (getIfCollided()){
      drivetrain.applyRequest(() -> brake);
    }

    angularVelocityWorldLog[i] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()); //TODO: test
    timeLog[i] = Utils.getCurrentTimeSeconds();
    i++;
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
    * VisionConstants.ACCELERATION_DUE_TO_GRAVITY // convert g-force to m/s^2
    - Math.cos(getPidgeonXYAccelerationDirection() + Math.PI/2) * getPigeonAngularAcceleration()
     * VisionConstants.DISTANCE_FROM_CENTER_TO_PIGEON;
  }

  /**
   * @return acceleration from pigeon in y direction is m/s^2
   */
  public double getPigeonAccelerationY(){  
    // use pythagrean theorum to find total acceleration
    return (drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorY().getValueAsDouble() // subtract gravity from acceleration
    * VisionConstants.ACCELERATION_DUE_TO_GRAVITY) // convert g-force to m/s^2
    - Math.sin(getPidgeonXYAccelerationDirection() + Math.PI/2) 
    * getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_PIGEON; 
  }

  public double getPidgeonXYAccelerationDirection(){
    if (drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() >= 0){
      return Math.tan(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() / 
      drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    } else {
      return Math.tan(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() / 
      drivetrain.getPigeon2().getAccelerationX().getValueAsDouble()) + Math.PI;
    }
  }

  public double getPigeonAngularAcceleration(){
    return (angularVelocityWorldLog[i-2] - angularVelocityWorldLog[i-1]) / (timeLog[i-2] - timeLog[i-1]);
  }


  public double getPigeonTotalAccelerationX(){
    return getPigeonAccelerationX() + getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE 
    * Math.cos(Math.tan(getPigeonAccelerationY() / getPigeonAccelerationX()) + Math.PI / 2);

  }

  public double getPigeonTotalAccelerationY(){
    return getPigeonAccelerationY() + getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE 
    * Math.sin(Math.tan(getPigeonAccelerationY() / getPigeonAccelerationX()) + Math.PI / 2);
  }

  public double getPrimitivePigeonAccelerationMagnitude(){
    return Math.hypot(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(), 
    drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());
  }

  public double getTotalPigeonAccelerationMagnitude() {
    return Math.hypot(getPigeonTotalAccelerationX(), getPigeonAccelerationY());
  }

  public double getTotalPigeonAccelerationDirection() {
    return Math.tan(getPigeonTotalAccelerationY() / getPigeonAccelerationX());
  }


// GET INFO FROM MOTOR

  /**
   * gets acceleration from a drivemotor of the specified module
   * @param moduleNumber
   * @return motor acceleration
   */
  public double getMotorAccelerationMagnitude(int moduleNumber){
    // get acceleration and convert to m/s^2
    return Units.rotationsToRadians(Math.abs(drivetrain.getModule(moduleNumber).getDriveMotor().getAcceleration().getValueAsDouble()) 
    * Units.inchesToMeters(TunerConstants.kWheelRadiusInches) / TunerConstants.kDriveGearRatio);
  }

  public double getMotorAccelerationDirection() {

    return Units.rotationsToRadians(drivetrain.getModule(0).getSteerMotor().getPosition().getValueAsDouble() 
    - Math.floor(drivetrain.getModule(0).getSteerMotor().getPosition().getValueAsDouble()));
  }

  // COMPARE MOTOR & PIGEON

  /**
   * compares acceleration of a specific drivemotor to pigeon and tolerance percentage in constants
   * @param moduleNumber
   * @return if motorAcceleration is within tolerance
   */
  public boolean checkMotorAcceleration(int moduleNumber){
    return Math.abs(getTotalPigeonAccelerationMagnitude() - getMotorAccelerationMagnitude(moduleNumber)) 
    > getMotorAccelerationMagnitude(moduleNumber) * VisionConstants.COLLISION_ACCELERATION_TOLERANCE_PERCENTAGE;
  }

  /**
   * compares acceleration of a specific drivemotor to pigeon and given tolerance
   * @param moduleNumber
   * @param tolerance
   * @return if motor is within tolerance
   */
  public boolean checkMotorAcceleration(int moduleNumber, double tolerance){
    return Math.abs(getTotalPigeonAccelerationMagnitude() - getMotorAccelerationMagnitude(moduleNumber)) > tolerance;
  }

  /**
   * @return if any motor has abnormal acceleration compared to pigeon and tolerance in constants
   */
  public boolean getIfCollided(){
    return checkMotorAcceleration(0) || checkMotorAcceleration(1) 
    || checkMotorAcceleration(2) || checkMotorAcceleration(3);
  }

  /**
   * @param tolerance
   * @return if any motor has abnormal acceleration compared to pigeon and given tolerance
   */
  public boolean getIfCollided(double tolerance){
    return checkMotorAcceleration(0, tolerance) || checkMotorAcceleration(1, tolerance) 
    || checkMotorAcceleration(2, tolerance) || checkMotorAcceleration(3, tolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}