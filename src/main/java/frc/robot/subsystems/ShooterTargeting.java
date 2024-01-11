// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterTargeting extends SubsystemBase {
  /** Creates a new ShooterTargeting. */
  Shooter shooter = new Shooter();
  double targetFlywheelRPM;
  double targetFlywheelAngle;
  double distanceFromTarget;
  public ShooterTargeting(

  ) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distanceFromTarget = getDistanceFromTarget();
    targetFlywheelAngle = clacShooterAngel();
    targetFlywheelRPM = clacShooterRPM();
  }

  public boolean readyToFire() {
    if (Math.abs(shooter.getFlywheelRPM() - targetFlywheelRPM) < ShooterConstants.SHOOTER_RPM_TOLERANCE && Math.abs(shooter.getFlywheelAngle() - targetFlywheelAngle) < ShooterConstants.SHOOTER_ANGLE_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  public double clacShooterRPM() {
    return ShooterConstants.SHOOTER_DISTANCE_SPEED_MAP.get(distanceFromTarget);
  }

  public double clacShooterAngel() {
    return ShooterConstants.SHOOTER_DISTANCE_ANGLE_MAP.get(distanceFromTarget);
  }

  public double getDistanceFromTarget() {
    return 0; //TODO make work
  }

  public double getTargetFlywheelRPM() {
    return targetFlywheelRPM;
  }
  
  public double getTargetFlywheelAngle() {
    return targetFlywheelAngle;
  }
}
