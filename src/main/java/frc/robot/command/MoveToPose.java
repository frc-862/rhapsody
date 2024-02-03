// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class MoveToPose extends Command {
  private final Pose2d target; 
  private final Swerve drivetrain;
  private FieldCentric drive; 
  /** Creates a new MoveToPose. */
  public MoveToPose(Pose2d target, Swerve drivetrain, SwerveRequest.FieldCentric drive) {
    this.drive = drive;
    this.target = target; 
    this.drivetrain = drivetrain; 
    addRequirements(drivetrain); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    Pose2d current = drivetrain.getPose().get(); 
    double dx = target.getTranslation().getX() - current.getTranslation().getX();
    double dy = target.getTranslation().getY() - current.getTranslation().getY();
    LightningShuffleboard.setDouble("MoveToPose", "dx", dx);
    LightningShuffleboard.setDouble("MoveToPose", "dy", dy);
    final double kp = 1.0; 
    drivetrain.applyRequest(()->drive.withVelocityX(dx*kp).withVelocityY(dy*kp));
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
