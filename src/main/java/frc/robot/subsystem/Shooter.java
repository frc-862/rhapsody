// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor1 = new TalonFX();
  TalonFX shooterMotor2 = new TalonFX();
  public Shooter() {

  }
  public void setRPM(double rpm){
    shooterMotor1.
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
