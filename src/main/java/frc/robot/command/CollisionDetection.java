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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;
public CollisionDetector collisionDetector;
SwerveRequest.SwerveDriveBrake brake;
public double[] angularVelocityWorldLog = {0d, 0d};
public double[] timeLog = {0d, 0d};
public double[] robotRotationFromMotor = {0d, 0d};
public double[] velocityXRequest = {0d, 0d};
public double[] velocityYRequest = {0d, 0d};
public double[] velocityRotRequest = {0d, 0d};
Pose2d storePoseWhenCollided;

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

    angularVelocityWorldLog[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    
    timeLog[1] = Utils.getCurrentTimeSeconds();

    robotRotationFromMotor[1] = drivetrain.getState().Pose.getRotation().getRadians();

    velocityXRequest[1] = drivetrain.getRequestX();
    velocityYRequest[1] = drivetrain.getRequestY();
    velocityRotRequest[1] = drivetrain.getRequestRot();
    

    LightningShuffleboard.setDouble("Collision Detection", "total pidgeon acceleration", 0d);
    LightningShuffleboard.setDouble("Collision Detection", "primitive pidgeon acceleration", 0d);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", 0d);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", 0d);
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", 0d);
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration direction", 0d);
    LightningShuffleboard.setBool("Collision Detection", "motor zero collided", false);
    LightningShuffleboard.setBool("Collision Detection", "collided", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // store ang vel and time to calc angular acceleration

    angularVelocityWorldLog[0] = angularVelocityWorldLog[1];
    timeLog[0] = timeLog[1];
    robotRotationFromMotor[0] = robotRotationFromMotor[1];
    velocityXRequest[0] = velocityXRequest[1];
    velocityYRequest[0] = velocityYRequest[1];
    velocityRotRequest[0] = velocityRotRequest[1];

    angularVelocityWorldLog[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    timeLog[1] = Utils.getCurrentTimeSeconds();
    robotRotationFromMotor[1] = drivetrain.getState().Pose.getRotation().getRadians();
    velocityXRequest[1] = drivetrain.getRequestX();
    velocityYRequest[1] = drivetrain.getRequestY();
    velocityRotRequest[1] = drivetrain.getRequestRot();

    LightningShuffleboard.setDouble("Collision Detection", "total pidgeon acceleration", getTotalPigeonAccelerationMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "primitive pidgeon acceleration", getPrimitivePigeonAccelerationMagnitude());
    LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", getTotalPigeonAccelerationDirection());
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAngularAcceleration() * VisionConstants.DISTANCE_FROM_CENTER_TO_MODULE);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon angular velocity", Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()));
    LightningShuffleboard.setDouble("Collision Detection", "yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
    LightningShuffleboard.setDouble("Collision Detection", "motor angular velocity", getMotorAngularVelocity());
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", getMotorAccelerationMagnitude(0));
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration direction", getMotorAccelerationDirection(0));
    LightningShuffleboard.setBool("Collision Detection", "motor zero collided", checkMotorAcceleration(0));
    LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided());
    LightningShuffleboard.setDouble("Collision Detection", "time", timeLog[0] - timeLog[1]);
    LightningShuffleboard.setDouble("Collision Detection", "get requested x velocity", drivetrain.getRequestX());
    LightningShuffleboard.setDouble("Collision Detection", "get requested y velocity", drivetrain.getRequestY());
    LightningShuffleboard.setDouble("Collision Detection", "get requested rot velocity", drivetrain.getRequestRot());
    LightningShuffleboard.setDouble("Collision Detection", "xy acceleration request", Math.hypot(getRequestXAcceleration(), getRequestYAcceleration()));
    
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
    return (angularVelocityWorldLog[1] - angularVelocityWorldLog[0]) / (timeLog[1] - timeLog[0]);
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

  /**
   * @return angular velocity from motor in radians/s
   */
  public double getMotorAngularVelocity(){
    return (robotRotationFromMotor[1] - robotRotationFromMotor[0]) / (timeLog[1] - timeLog[0]);
  }

  // GET RECQUESTED INFO

  public double getRequestXAcceleration(){
    return (velocityXRequest[1] - velocityXRequest[0]) / (timeLog[1] - timeLog[0]);
  }

  public double getRequestYAcceleration(){
    return (velocityYRequest[1] - velocityYRequest[0]) / (timeLog[1] - timeLog[0]);
  }

  public double getRequestRotAcceleration(){
    return (velocityRotRequest[1] - velocityRotRequest[0]) / (timeLog[1] - timeLog[0]);
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
    && getTotalPigeonAccelerationDirection() - getMotorAccelerationDirection(moduleNumber) 
    < VisionConstants.COLLISION_ACCELERATION_DIRECTION_TOLERANCE;
  }

  /**
   * compares acceleration of a specific drivemotor to pigeon and given tolerance
   * @param moduleNumber
   * @return if motor is within tolerance
   */
  public boolean checkMotorAcceleration(int moduleNumber, double magnitudeTolerance, double directionTolerance){
    return Math.abs(getTotalPigeonAccelerationMagnitude() - getMotorAccelerationMagnitude(moduleNumber)) > magnitudeTolerance
    && getTotalPigeonAccelerationDirection() - getMotorAccelerationDirection(moduleNumber) 
    < directionTolerance;
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
   * @param directionTolerance
   * @return if any motor has abnormal acceleration compared to pigeon and given tolerances
   */
  public boolean getIfCollided(double magnitudeTolerance, double directionTolerance){
    return checkMotorAcceleration(0, magnitudeTolerance, directionTolerance) 
    || checkMotorAcceleration(1, magnitudeTolerance, directionTolerance) 
    || checkMotorAcceleration(2, magnitudeTolerance, directionTolerance) 
    || checkMotorAcceleration(3, magnitudeTolerance, directionTolerance);
  }
  
// DO STUFF WHEN COLLIDED

  public void storePose(){
    storePoseWhenCollided = drivetrain.getPose2d();
  }
  // failsafe mode

  // figure out how to reorient


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}