// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.FalconConfig;

public class Climber extends SubsystemBase {
  /** Creates a new Climb. */

  // create variables
  public TalonFX climbMotorR;
  public TalonFX climbMotorL;
  private double setPoint;

  public Climber() {
    // configure climb motors
    climbMotorR = FalconConfig.createMotor(CAN.CLIMB_RIGHT, CAN.CANBUS_FD,
      ClimbConstants.CLIMB_RIGHT_MOTOR_INVERT, 
      ClimbConstants.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT, 
      ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, 
      ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, ClimbConstants.CLIMB_MOTOR_KP,
      ClimbConstants.CLIMB_MOTOR_KI, ClimbConstants.CLIMB_MOTOR_KD, 
      ClimbConstants.CLIMB_MOTOR_KS, ClimbConstants.CLIMB_MOTOR_KV);
    climbMotorL = FalconConfig.createMotor(CAN.CLIMB_LEFT, CAN.CANBUS_FD,
      ClimbConstants.CLIMB_LEFT_MOTOR_INVERT, 
      ClimbConstants.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT, 
      ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, 
      ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, ClimbConstants.CLIMB_MOTOR_KP,
      ClimbConstants.CLIMB_MOTOR_KI, ClimbConstants.CLIMB_MOTOR_KD, 
      ClimbConstants.CLIMB_MOTOR_KS, ClimbConstants.CLIMB_MOTOR_KV);
  }

  /**
   * sets power to both climb motors
   * @param power
   */
  public void setPower(double power) {
      climbMotorR.set(power);
      climbMotorL.set(power);
  }

  /**
   * stops all climb motors
   */
  public void stopClimb(){
    setPower(0);
  }

  /**
   * sets a setpoint that climb will go to using pid
   */
  public void setSetPoint(double setPoint){
    this.setPoint = setPoint;
  }

  /**
   * @return current climb pid setpoint
   */
  public double getSetPoint(){
    return setPoint;
  }

  /**
   * @return height of climb
   */
  public double getHeight(){
    return climbMotorR.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}