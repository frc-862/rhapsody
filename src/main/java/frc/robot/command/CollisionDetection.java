// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollisionConstants;
import frc.robot.subsystems.CollisionDetector;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class CollisionDetection extends Command {
  /** Creates a new CollisionDetection. */
  
public Swerve drivetrain;
public CollisionDetector collisionDetector;
public double[] rotVelP = {0d, 0d};
public double[] time = {0d, 0d};
public double[] xVelC = {0d, 0d};
public double[] xVelY = {0d, 0d};
public double[] rotVelC = {0d, 0d};

public LinearFilter xAccCFilter = LinearFilter.movingAverage(50);
public LinearFilter yAccCFilter = LinearFilter.movingAverage(50);
public LinearFilter rotAccCFilter = LinearFilter.movingAverage(50);;

  public CollisionDetection(Swerve drivetrain, CollisionDetector collisionDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.collisionDetector = collisionDetector;
    addRequirements(collisionDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storeVelocities();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storeVelocities();

    LightningShuffleboard.setDouble("Collision Detection", "pidgeon acceleration magnitude", getPigeonAcceleration()[2]);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", getPigeonAcceleration()[3]);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAcceleration()[4]);
    LightningShuffleboard.setDouble("Collision Detection", "pigeon angular velocity", Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()));
    LightningShuffleboard.setDouble("Collision Detection", "yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
    LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude(exp filter)", getChassisAcceleration()[2]);
    LightningShuffleboard.setDouble("Collision Detection", "motor angular acceleration(exp filter)", getChassisAcceleration()[4]);
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
    xVelY[0] = xVelY[1];
    rotVelC[0] = rotVelC[1];

    rotVelP[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    time[1] = Utils.getCurrentTimeSeconds();
    xVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
    xVelY[1] = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
    rotVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
  }

  /**
   * @return array with pigeon acceleration 
   * 0 - Acceleration in X direction
   * 1 - Acceleration in Y direction
   * 2 - Magnitude of X and Y Acceleration
   * 3 - Direction of X and Y Acceleration
   * 4 - Rotational Acceleration
   */
  public double[] getPigeonAcceleration(){
    double accX = drivetrain.getPigeon2().getAccelerationX().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorX().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

    double accY = drivetrain.getPigeon2().getAccelerationY().getValueAsDouble() 
    - drivetrain.getPigeon2().getGravityVectorY().getValueAsDouble() // subtract gravity from acceleration
    * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

    double accMag = Math.hypot(accX, accY);
    double accDirection = Math.atan2(accY, accX);
    double accRot = (rotVelP[1] - rotVelP[0]) / (time[1] - time[0]);
    return new double[] {accX, accY, accMag, accDirection, accRot};
  }

  /**
   * @return array with chassis acceleration 
   * 0 - Acceleration in X direction
   * 1 - Acceleration in Y direction
   * 2 - Magnitude of X and Y Acceleration
   * 3 - Direction of X and Y Acceleration
   * 4 - Rotational Acceleration
   */
  public double[] getChassisAcceleration(){
    double timeDelta = time[1] - time[0];
    double accX = xAccCFilter.calculate((xVelC[1] - xVelC[0]) / timeDelta); // calculate acceleration in x direction and filter
    double accY = yAccCFilter.calculate((xVelY[1] - xVelY[0]) / timeDelta); // calculate acceleration in y direction and filter
    double accMag = Math.hypot(accX, accY);
    double accDirection = Math.atan2(accY, accX);
    double accRot = rotAccCFilter.calculate((rotVelC[1] - rotVelC[0]) / timeDelta); // calculate rot acceleration and filter
    return new double[] {accX, accY, accMag, accDirection, accRot};
  }

  /**
   * Compare Pigeon and Chassis Acceleraiton
   * @return boolean array
   * 0 - check x
   * 1 - check y
   * 2 - check rot
   * 3 - check all
   */
  public boolean[] getIfCollided(){
    double differenceX = Math.abs(getPigeonAcceleration()[0] - getChassisAcceleration()[0]);
    boolean xCollided = differenceX > getPigeonAcceleration()[0] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceX > CollisionConstants.ACCELERATION_TOLERANCE;

    double differenceY = Math.abs(getPigeonAcceleration()[1] - getChassisAcceleration()[1]);
    boolean yCollided = differenceY > getPigeonAcceleration()[1] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceY > CollisionConstants.ACCELERATION_TOLERANCE;

    double differenceRot = Math.abs(getPigeonAcceleration()[4] - getChassisAcceleration()[4]);
    boolean rotCollided = differenceRot > getPigeonAcceleration()[4] * CollisionConstants.ACCELERATION_TOLERANCE 
    && differenceX > CollisionConstants.ACCELERATION_TOLERANCE;

    boolean collided = xCollided || yCollided || rotCollided;

    return new boolean[] {xCollided, yCollided, rotCollided, collided};
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}