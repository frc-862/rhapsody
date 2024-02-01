// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDCommands extends SubsystemBase {
  private LEDs leds;

  /**
   * @param leds used to set leds
   */
  public LEDCommands(LEDs leds) {
    this.leds = leds;
  }

  @Override
  public void periodic() {}

  /**
   * @param segmentSize size of each color segment
   */
  public void swirl(int segmentSize) {
    for (int i = 0; i < LEDsConstants.LED_BUFFER_TIME; i++ ) {
        if (((i + (int)Timer.getFPGATimestamp()) / segmentSize) % 2 == 0) {
            leds.setIndexRGB(i,0,0,255);
        } else {
            leds.setIndexRGB(i,255,35,0);
        }
    }
  }

  /**
   * @param speedMulti higher values make blink faster, max value ~50
   */
  public void blink(double speedMulti) {
    if ((int)Timer.getFPGATimestamp() * speedMulti % 2 == 0) {
      leds.setSolidRGB(0, 255, 0);
    } else {
      leds.setSolidRGB(0, 0, 0);
    }
  }

  public void pulse() {
    leds.setSolidHSV(240, 255, (int)Math.abs((Math.sin(Timer.getFPGATimestamp() * 2) * 255)));
  }
}