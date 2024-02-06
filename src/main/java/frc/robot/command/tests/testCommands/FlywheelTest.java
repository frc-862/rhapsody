// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlywheelTest extends Command {

  private Flywheel flywheel;
  private double motor2Speed;
  private double motor1Speed;

  public FlywheelTest(Flywheel flywheel, double motor1Speed, double motor2Speed) {
    this.flywheel = flywheel;
    this.motor1Speed = motor1Speed;
    this.motor2Speed = motor2Speed;

    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    flywheel.setTopMoterRPM(motor1Speed);
    flywheel.setTopMoterRPM(motor2Speed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.setAllMotorsRPM(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
