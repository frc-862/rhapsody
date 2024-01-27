// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.LEDs;

public class SetLED extends Command {
  private LEDs leds;
  private Collector collector;
  /**
     * Sets the angle of the pivot
     * 
     * @param leds set the leds
     * @param collector find if collector has peice
     */
  public SetLED(LEDs leds, Collector collector) {
    this.leds = leds;
    this.collector = collector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collector.hasPiece()) { 
      leds.setSolidRGB(0, 255, 0);
    } else {
      leds.setSolidRGB(255, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
