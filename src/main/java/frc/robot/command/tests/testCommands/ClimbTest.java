// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimbConstants;

public class ClimbMotorTest extends Command {
  
  private Climber climber;
  private double power;

  /**
   * System test command for testing climb motors
   * @param climber climber subsystem 
   * @param power power to control up or down
   */
  public ClimbMotorTest(Climber climber, double power) {
    this.climber = climber;
    this.power = power;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPower(ClimbConstants.CLIMB_TEST_POWER * power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}