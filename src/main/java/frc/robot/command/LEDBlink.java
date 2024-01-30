// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class LEDBlink extends Command {
  private LEDs leds;
  private double blinkTime;
  private double startTime;
  private int LEDHue;
  /** Creates a new LEDBlink. */
  public LEDBlink(LEDs leds, int blinkTime, int LEDHue) {
    this.leds = leds;
    this.LEDHue = LEDHue;
    this.blinkTime = blinkTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime % blinkTime == 0) {
      leds.setSolidHSV(LEDHue, 100, 255);
    } else {
      leds.setSolidHSV(240, 255, 255);
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
