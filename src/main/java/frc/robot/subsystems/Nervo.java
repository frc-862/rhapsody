// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.PWM;

public class Nervo extends SubsystemBase {
  private Servo fireServo;
  private Servo flywheelServo;
  public Nervo() {
    fireServo = new Servo(PWM.FIRE_SERVO_PORT);
    flywheelServo = new Servo(PWM.FLYWHEEL_SERVO_PORT);
  }

  @Override
  public void periodic() {}

  public Command fireServo() {
    return new StartEndCommand(() -> {
      fireServo.set(1);}, 
			() -> {
        fireServo.set(0);
      });
  }

  public Command flywheelServo() {
    return new StartEndCommand(() -> {
      flywheelServo.set(1);}, 
			() -> {
        flywheelServo.set(0);
      });
  }
}
