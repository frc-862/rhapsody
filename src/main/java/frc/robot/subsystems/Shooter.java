// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.thunder.config.FalconConfig;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor1;
  TalonFX shooterMotor2;
  TalonFX angleMotor;
  ShooterTargeting shooterTargeting = new ShooterTargeting();
  
  final VelocityVoltage rpmTarget = new VelocityVoltage(0).withSlot(0);
  final VelocityVoltage angleTarget = new VelocityVoltage(0).withSlot(0);

  
  public Shooter() {
    shooterMotor1 = FalconConfig.createMotor(CAN.SHOOTING_MOTOR_1, getName(), ShooterConstants.SHOOTING_MOTOR_1_INVERT, ShooterConstants.SHOOTING_MOTOR_SUPPLY_CURRENT_LIMIT, ShooterConstants.SHOOTING_MOTOR_STATOR_CURRENT_LIMIT, ShooterConstants.SHOOTING_MOTOR_NEUTRAL_MODE, ShooterConstants.SHOOTING_MOTOR_KP, ShooterConstants.SHOOTING_MOTOR_KI, ShooterConstants.SHOOTING_MOTOR_KD, ShooterConstants.SHOOTING_MOTOR_KS, ShooterConstants.SHOOTING_MOTOR_KV);
    shooterMotor2 = FalconConfig.createMotor(CAN.SHOOTING_MOTOR_2, getName(), ShooterConstants.SHOOTING_MOTOR_2_INVERT, ShooterConstants.SHOOTING_MOTOR_SUPPLY_CURRENT_LIMIT, ShooterConstants.SHOOTING_MOTOR_STATOR_CURRENT_LIMIT, ShooterConstants.SHOOTING_MOTOR_NEUTRAL_MODE, ShooterConstants.SHOOTING_MOTOR_KP, ShooterConstants.SHOOTING_MOTOR_KI, ShooterConstants.SHOOTING_MOTOR_KD, ShooterConstants.SHOOTING_MOTOR_KS, ShooterConstants.SHOOTING_MOTOR_KV);
  }
  public void setRPM(double rpm){
    shooterMotor1.setControl(rpmTarget.withVelocity(rpm).withFeedForward(0.5));
    shooterMotor2.setControl(rpmTarget.withVelocity(rpm).withFeedForward(0.5));
  }

  public void setAngle(double angle) {
    angleMotor.setControl(angleTarget.withVelocity(angle).withFeedForward(0.5));
  }

  public double getFlywheelRPM() {
    return (shooterMotor1.getVelocity().getValue() + shooterMotor2.getVelocity().getValue()) * 30;
  }

  public double getFlywheelAngle() {
    return 0; //TODO make work
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
