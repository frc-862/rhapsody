// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollisionConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */

  public Swerve drivetrain;
  public Rotation2d collisionDirection;
  // init arrays for calculating accelerations
  public double[] rotVelP = {0d, 0d};
  public double[] time = {0d, 0d};
  public double[] xVelC = {0d, 0d};
  public double[] yVelC = {0d, 0d};
  public double[] rotVelC = {0d, 0d};
  // filters for filtering acceleration values
  public LinearFilter xAccCFilter = LinearFilter.movingAverage(50);
  public LinearFilter yAccCFilter = LinearFilter.movingAverage(50);
  public LinearFilter rotAccCFilter = LinearFilter.movingAverage(50);

  public CollisionDetection(Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storeVelocities(); // store initial velocities to avoid null values
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storeVelocities();
    // log values to shuffleboard
    LightningShuffleboard.setDouble("Collision Detection", "pidgeon acceleration magnitude", getPigeonAcceleration()[2]);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", getPigeonAcceleration()[3]);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAcceleration()[4]);
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", getChassisAcceleration()[2]);
    LightningShuffleboard.setDouble("Collision Detection", "motor angular acceleration", getChassisAcceleration()[4]);
    LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided()[3]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /**
   * log velocities to calculate acceleration
   */
  public void storeVelocities(){
    rotVelP[0] = rotVelP[1];
    time[0] = time[1];
    xVelC[0] = xVelC[1];
    yVelC[0] = yVelC[1];
    rotVelC[0] = rotVelC[1];
    // store velocities to calculate acceleration
    rotVelP[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    time[1] = Utils.getCurrentTimeSeconds();
    xVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond / TunerConstants.kDriveGearRatio;
    yVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond / TunerConstants.kDriveGearRatio;
    rotVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond / TunerConstants.kDriveGearRatio;
  }

  /**
   * @return array with pigeon acceleration 
   * <li> 0 - Acceleration in X direction
   * <li> 1 - Acceleration in Y direction
   * <li> 2 - Magnitude of X and Y Acceleration
   * <li> 3 - Direction of X and Y Acceleration
   * <li> 4 - Rotational Acceleration
   */
  public double[] getPigeonAcceleration(){
    double accX = drivetrain.getPigeon2().getAccelerationX().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorX().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

    double accY = drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorY().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

    double accMag = Math.hypot(accX, accY); // calculate magnitude of acceleration
    double accDirection = Math.atan2(accY, accX); // calculate direction of acceleration
    double accRot = (rotVelP[1] - rotVelP[0]) / (time[1] - time[0]); // calculate rotational acceleration
    return new double[] {accX, accY, accMag, accDirection, accRot};
  }

  /**
   * @return array with chassis acceleration 
   * <li> 0 - Acceleration in X direction
   * <li> 1 - Acceleration in Y direction
   * <li> 2 - Magnitude of X and Y Acceleration
   * <li> 3 - Direction of X and Y Acceleration
   * <li> 4 - Rotational Acceleration
   */
  public double[] getChassisAcceleration(){
    double deltaTime = time[1] - time[0];
    double accX = xAccCFilter.calculate((xVelC[1] - xVelC[0]) / deltaTime); // calculate acceleration in x direction and filter
    double accY = yAccCFilter.calculate((yVelC[1] - yVelC[0]) / deltaTime); // calculate acceleration in y direction and filter
    double accMag = Math.hypot(accX, accY); // calculate magnitude of acceleration
    double accDirection = Math.atan2(accY, accX); // calculate direction of acceleration
    double accRot = rotAccCFilter.calculate((rotVelC[1] - rotVelC[0]) / deltaTime); // calculate rot acceleration and filter
    return new double[] {accX, accY, accMag, accDirection, accRot};
  }

  /**
   * Compare Pigeon and Chassis Acceleraiton
   * @return boolean array
   * <li> 0 - check x
   * <li> 1 - check y
   * <li> 2 - check rot
   * <li> 3 - check all
   */
  public boolean[] getIfCollided(){
    double differenceX = Math.abs(getPigeonAcceleration()[0] - getChassisAcceleration()[0]); // calculate difference in x acceleration
    boolean xCollided = differenceX > getPigeonAcceleration()[0] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceX > CollisionConstants.MIN_ACCELERATION_DIFF; // check if difference is greater than tolerance and min difference

    double differenceY = Math.abs(getPigeonAcceleration()[1] - getChassisAcceleration()[1]);
    boolean yCollided = differenceY > getPigeonAcceleration()[1] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceY > CollisionConstants.MIN_ACCELERATION_DIFF;

    double differenceRot = Math.abs(getPigeonAcceleration()[4] - getChassisAcceleration()[4]);
    boolean rotCollided = differenceRot > getPigeonAcceleration()[4] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceX > CollisionConstants.MIN_ACCELERATION_DIFF;

    collisionDirection = new Rotation2d(Math.atan2(differenceY, differenceX));
    boolean collided = xCollided || yCollided || rotCollided;

    return new boolean[] {xCollided, yCollided, rotCollided, collided};
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}