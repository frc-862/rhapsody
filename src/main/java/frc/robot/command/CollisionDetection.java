// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.Arrays;

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

    // store ang vel and time to calc angular acceleration

    angularVelocityWorldLog = Arrays.copyOf(angularVelocityWorldLog, angularVelocityWorldLog.length + 1);
    angularVelocityWorldLog[angularVelocityWorldLog.length - 1] 
    = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

    timeLog = Arrays.copyOf(timeLog, timeLog.length + 1);
    timeLog[timeLog.length - 1] = Utils.getCurrentTimeSeconds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // store ang vel and time to calc angular acceleration

    angularVelocityWorldLog = Arrays.copyOf(angularVelocityWorldLog, angularVelocityWorldLog.length + 1);
    angularVelocityWorldLog[angularVelocityWorldLog.length - 1] 
    = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

    timeLog = Arrays.copyOf(timeLog, timeLog.length + 1);
    timeLog[timeLog.length - 1] = Utils.getCurrentTimeSeconds();

    LightningShuffleboard.setDouble("Collision Detection", "total pidgeon acceleration", getTotalPigeonAccelerationMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "primitive pidgeon acceleration", getPrimitivePigeonAccelerationMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "pigeonAccelarationDirection", getTotalPigeonAccelerationDirection());
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE);
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", getMotorAccelerationMagnitude(0));
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration direction", getMotorAccelerationDirection(0));
    LightningShuffleboard.setBool("Collision Detection", "motor zero collided", checkMotorAcceleration(0));
    LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided());
    
    if (getIfCollided()){
      // drivetrain.applyRequest(() -> brake);&& getTotalPigeonAccelerationDirection() - getMotorAccelerationDirection(moduleNumber) < VisionConstants.COLLISION_ACCELERATION_DIRECTION_TOLERANCE
    }
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
    - Math.cos(getPidgeonXYAccelerationDirection() + Math.PI/2) * getPigeonAngularAcceleration() // subtract ang vel of pigeon on robot
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
    * getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_PIGEON; // subtract ang vel from of pigeon on robot
  }

  /**
   * @return direction of x/y acceleration in radians
   */
  public double getPidgeonXYAccelerationDirection(){
    if (drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() >= 0){
      return Math.tan(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() / 
      drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    } else {
      return Math.tan(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() / 
      drivetrain.getPigeon2().getAccelerationX().getValueAsDouble()) + Math.PI;
    }
  }

  /**
   * @return pigeon angular velocity in radians per second
   */
  public double getPigeonAngularAcceleration(){
    return (angularVelocityWorldLog[angularVelocityWorldLog.length - 1] - angularVelocityWorldLog[angularVelocityWorldLog.length]) 
    / (timeLog[timeLog.length - 1] - timeLog[timeLog.length]);
  }


  /**
   * add angular velocity to x acceleration
   * @return total pigeon acceleration in x direction in m/s^2
   */
  public double getPigeonTotalAccelerationX(){
    return getPigeonAccelerationX() + getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE 
    * Math.cos(Math.tan(getPigeonAccelerationY() / getPigeonAccelerationX()) + Math.PI / 2);

  }
  /**
   * add angular velocity to y acceleration
   * @return total pigeon acceleration in y direction in m/s^2
   */
  public double getPigeonTotalAccelerationY(){
    return getPigeonAccelerationY() + getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE 
    * Math.sin(Math.tan(getPigeonAccelerationY() / getPigeonAccelerationX()) + Math.PI / 2);
  }

  /**
   * @return primitive acceleration magnitude in m/s^2
   */
public double getPrimitivePigeonAccelerationMagnitude(){
    return Math.hypot(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(), 
    drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());
  }

  /**
   * @return total acceleration magnitude in m/s^2
   */
  public double getTotalPigeonAccelerationMagnitude() {
    return Math.hypot(getPigeonTotalAccelerationX(), getPigeonAccelerationY());
  }

  /**
   * @return total acceleration direction in radians
   */
public double getTotalPigeonAccelerationDirection() {
    return Math.tan(getPigeonTotalAccelerationY() / getPigeonAccelerationX());
  }


// GET INFO FROM MOTOR

  /**
   * gets acceleration magnitude from a drivemotor of the specified module
   * @param moduleNumber
   * @return motor acceleration magnitude
   */
  public double getMotorAccelerationMagnitude(int moduleNumber){
    // get acceleration magnitude and convert to m/s^2
    return Units.rotationsToRadians(Math.abs(drivetrain.getModule(moduleNumber).getDriveMotor().getAcceleration().getValueAsDouble()) 
    * Units.inchesToMeters(TunerConstants.kWheelRadiusInches) / TunerConstants.kDriveGearRatio);
  }

  /**
   * gets acceleration direction from a drivemotor of the specified module
   * @param moduleNumber
   * @return motor acceleration direction
   */
  public double getMotorAccelerationDirection(int moduleNumber) {

    return Units.rotationsToRadians(drivetrain.getModule(moduleNumber).getSteerMotor().getPosition().getValueAsDouble() 
    - Math.floor(drivetrain.getModule(moduleNumber).getSteerMotor().getPosition().getValueAsDouble()));
  }

  // COMPARE MOTOR & PIGEON

  /**
   * compares acceleration of a specific drivemotor to pigeon and tolerance percentage in constants
   * @param moduleNumber
   * @return if motorAcceleration is within tolerance
   */
  public boolean checkMotorAcceleration(int moduleNumber){
    return Math.abs(getTotalPigeonAccelerationMagnitude() - getMotorAccelerationMagnitude(moduleNumber)) 
    > getMotorAccelerationMagnitude(moduleNumber) * VisionConstants.COLLISION_ACCELERATION_MAGNITUDE_TOLERANCE_PERCENTAGE
    && getTotalPigeonAccelerationDirection() - getMotorAccelerationDirection(moduleNumber) < VisionConstants.COLLISION_ACCELERATION_DIRECTION_TOLERANCE;
  }

  /**
   * compares acceleration of a specific drivemotor to pigeon and given tolerance
   * @param moduleNumber
   * @param tolerance
   * @return if motor is within tolerance
   */
  public boolean checkMotorAcceleration(int moduleNumber, double tolerance){
    return Math.abs(getTotalPigeonAccelerationMagnitude() - getMotorAccelerationMagnitude(moduleNumber)) > tolerance
    && getTotalPigeonAccelerationDirection() - getMotorAccelerationDirection(moduleNumber) 
    < VisionConstants.COLLISION_ACCELERATION_DIRECTION_TOLERANCE;
  }

  /**
   * @return if any motor has abnormal acceleration compared to pigeon and tolerance in constants
   */
  public boolean getIfCollided(){
    return checkMotorAcceleration(0) || checkMotorAcceleration(1) 
    || checkMotorAcceleration(2) || checkMotorAcceleration(3);
  }

  /**
   * @param magnitudeTolerance
   * @return if any motor has abnormal acceleration compared to pigeon and given tolerance
   */
  public boolean getIfCollided(double magnitudeTolerance){
    return checkMotorAcceleration(0, magnitudeTolerance) || checkMotorAcceleration(1, magnitudeTolerance) 
    || checkMotorAcceleration(2, magnitudeTolerance) || checkMotorAcceleration(3, magnitudeTolerance);
  }
  
// DO STUFF WHEN COLLIDED

  // failsafe mode

  // figure out how to reorient


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}