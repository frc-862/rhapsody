// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDs;

public class SwirlLEDs extends Command {
  private LEDs leds;
  private int index = 0;
  private int segmentLength = 5;
  
  /**
   * Creats a new SwirlLEDs command
   * @param leds set the leds
   */
  public SwirlLEDs(LEDs leds) {
    this.leds = leds;

    addRequirements(leds);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
   for (int i = 0; i < LEDsConstants.LED_BUFFER_TIME - segmentLength; i+= segmentLength) {
    if (i % segmentLength * 2 == 0) { 
      for (int x = 0; x < segmentLength; x++) {
        leds.setIndexHSV(i + x + index, 30, 255, 255);
      }
    } else {
      for (int x = 0; x < segmentLength; x++) {
        leds.setIndexHSV(i + x + index, 240, 255, 255);
      }
    }
  }
  for (int i = LEDsConstants.LED_BUFFER_TIME + index - segmentLength; i < LEDsConstants.LED_BUFFER_TIME; i++) {
    leds.setIndexHSV(i, 240, 255, 255);
  }
  for (int i = 0; i < index; i++) {
    leds.setIndexHSV(i, 240, 255, 255);
  }
  index++;
  index %= segmentLength + 1;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
