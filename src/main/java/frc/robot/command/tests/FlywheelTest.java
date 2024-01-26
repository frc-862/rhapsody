// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlywheelTest extends Command {
  private Flywheel flywheel;
  private double motor2Speed;
  private double motor1Speed;

  /** Creates a new ShootTest. */
  public FlywheelTest(Flywheel flywheel, double motor1Speed, double motor2Speed) {
    this.flywheel = flywheel;
    this.motor1Speed = motor1Speed;
    this.motor2Speed = motor2Speed;
    
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setMoter1RPM(motor1Speed);
    flywheel.setMoter1RPM(motor2Speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setAllMotorsRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
