// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class SwirlLEDs extends Command {
  private LEDs leds;
  private int index = 0;
  /** Creates a new LEDs. *//**
     * Sets the angle of the pivot
     * 
     * @param leds set the leds
     */
  public SwirlLEDs(LEDs leds) {
    this.leds = leds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   for (int i = 0; i < 55; i+= 5) {
    if (i % 10 == 0) { 
      for (int x = 0; x < 5; x++) {
        leds.setIndexHSV(i + x + index, 30, 255, 255);
      }
    } else {
      for (int x = 0; x < 5; x++) {
        leds.setIndexHSV(i + x + index, 240, 255, 255);
      }
    }
  }
  for (int i = 55+index; i < 60; i++) {
    leds.setIndexHSV(i, 240, 255, 255);
  }
  for (int i = 0; i < index; i++) {
    leds.setIndexHSV(i, 240, 255, 255);
  }
  index ++;
  index %= 6;
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
