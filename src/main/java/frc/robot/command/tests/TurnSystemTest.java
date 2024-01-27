// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TurnSystemTest extends Command {
  
  private Swerve drivetrain;
  private DoubleSupplier speed;

  /**
   * System test for testing azimuth motors
   * @param drivetrain swerve subsystem
   * @param speed rotational rate
   */
  public TurnSystemTest(Swerve drivetrain, DoubleSupplier speed) {
    this.drivetrain = drivetrain;
    this.speed = speed;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(speed.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
