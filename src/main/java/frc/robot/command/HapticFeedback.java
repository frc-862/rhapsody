// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class HapticFeedback extends Command {
  private XboxController driver;
  private double value;

  /** Creates a new HapticFeedback. */
  /*
   * @param driver the controller to vibrate
   * @param value the strength of the vibration
   */
  public HapticFeedback(XboxController driver, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.value = value;
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driver.setRumble(GenericHID.RumbleType.kRightRumble, value);
    driver.setRumble(GenericHID.RumbleType.kLeftRumble, value);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driver.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
