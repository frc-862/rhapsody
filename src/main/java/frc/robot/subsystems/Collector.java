// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;;

public class Collector extends SubsystemBase {

  // Declare collector hardware
  private TalonFX collectorMotor;
  private DigitalInput collectorEntryBeamBreak;

  public Collector() {
    // Initialize collector hardware
    collectorMotor = new TalonFX(CollectorConstants.COLLECTOR_MOTOR);
    collectorEntryBeamBreak = new DigitalInput(CollectorConstants.COLLECTOR_ENTRY_BEAMBREAK);
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
