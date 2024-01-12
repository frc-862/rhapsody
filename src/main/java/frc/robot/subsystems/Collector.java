// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.config.FalconConfig;

public class Collector extends SubsystemBase {

  // Declare collector hardware
  private TalonFX collectorMotor;
  private DigitalInput collectorEntryBeamBreak;

  public Collector() {
    // Initialize collector hardware
    collectorMotor = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR, getName(), CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT, CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT, CollectorConstants.COLLECTOR_MOTOR_NEUTRAL_MODE);
    collectorEntryBeamBreak = new DigitalInput(RobotMap.COLLECTOR_ENTRY_BEAMBREAK);
  }

  @Override
  public void periodic() {
    
  }

  /**
   * @return When an object is present, returns false, otherwise returns true
   */
  public boolean getEntryBeamBreakState() {
    return collectorEntryBeamBreak.get();
  }

  /**
   * Sets the power of the collector motor
   * @param power Double value from -1.0 to 1.0 (positive collects inwards)
   */
  public void setPower(double power) {
    collectorMotor.set(power);
  }

  /**
   * Stops the collector
   */
  public void stop() {
    setPower(0d);
  }

}
