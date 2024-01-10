// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.FalconConfig;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor1;
  TalonFX shooterMotor2;
  
  final VelocityVoltage rpmTarget = new VelocityVoltage(0).withSlot(0);
  
  public Shooter() {
    shooterMotor1 = FalconConfig.createMotor(CAN.SHOOTER_MOTOR_1, getName(), ShooterConstants.MOTOR_1_INVERT, ShooterConstants.NEUTRAL_MODE, ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KS, ShooterConstants.KV);
    shooterMotor2 = FalconConfig.createMotor(CAN.SHOOTER_MOTOR_2, getName(), ShooterConstants.MOTOR_2_INVERT, ShooterConstants.NEUTRAL_MODE, ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KS, ShooterConstants.KV);
  }
  public void setRPM(double rpm){
    shooterMotor1.setControl(rpmTarget.withVelocity(rpm).withFeedForward(0.5));
    shooterMotor2.setControl(rpmTarget.withVelocity(rpm).withFeedForward(0.5));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
