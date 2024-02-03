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
  private boolean finished;

  /** Creates a new MoveToPose. */
  public MoveToPose(Pose2d target, Swerve drivetrain, SwerveRequest.FieldCentric drive) {
    this.drive = drive;
    this.target = target; 
    this.drivetrain = drivetrain; 
    addRequirements(drivetrain); 
    // Use addRequirements() here to declare subsystem dependencies.
    LightningShuffleboard.setDouble("MoveToPose", "dx", 0);
    LightningShuffleboard.setDouble("MoveToPose", "dy", 0);
    LightningShuffleboard.setDouble("MoveToPose", "targetX", target.getX());
    LightningShuffleboard.setDouble("MoveToPose", "targetY", target.getY());
    LightningShuffleboard.setDouble("MoveToPose", "currentX", 0);
    LightningShuffleboard.setDouble("MoveToPose", "currentY", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    Pose2d current = drivetrain.getPose().get(); 
    double dx = target.getTranslation().getX() - current.getTranslation().getX();
    double dy = target.getTranslation().getY() - current.getTranslation().getY();
    LightningShuffleboard.setDouble("MoveToPose", "dx", dx);
    LightningShuffleboard.setDouble("MoveToPose", "dy", dy);
    LightningShuffleboard.setDouble("MoveToPose", "targetX", target.getX());
    LightningShuffleboard.setDouble("MoveToPose", "targetY", target.getY());
    LightningShuffleboard.setDouble("MoveToPose", "currentX", current.getX());
    LightningShuffleboard.setDouble("MoveToPose", "currentY", current.getY());
    final double kp = 0.3; 
    final double minSpeed = 0.1;

    double powerx = dx * kp;
    double powery = dy * kp;

    if (minSpeed > Math.abs(powerx)) {
      powerx = minSpeed * Math.signum(dx);
    }

    if (minSpeed > Math.abs(powery)) {
      powery = -minSpeed * -Math.signum(dy);
    }

    var dist = Math.sqrt(dx*dx + dy*dy);
    if (dist < 0.1) {
      powerx = 0;
      powery = 0;
      finished = true;
    }
    drivetrain.setControl(drive.withVelocityX(powerx).withVelocityY(powery));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
