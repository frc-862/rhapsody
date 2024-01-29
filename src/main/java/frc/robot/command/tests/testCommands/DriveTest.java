// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveTest extends Command {

  private Swerve drivetrain;
  private DoubleSupplier speedX;
  private DoubleSupplier speedY;

  /**
   * System test command for testing drive motors
   * @param drivetrain swerve subsystem
   * @param speedX X velocity
   * @param speedY Y velocity
   */
  public DriveTest(Swerve drivetrain, DoubleSupplier speedX, DoubleSupplier speedY) {
    this.drivetrain = drivetrain;
    this.speedX = speedX;
    this.speedY = speedY;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(speedX.getAsDouble()).withVelocityY(speedY.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
