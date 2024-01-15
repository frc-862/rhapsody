// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.thunder.vision.Limelight;

public class AngleTowardTags extends Command {

  private Swerve drivetrain;
  private Limelight[] limelights;

  public Pose2d targetPoseRobotSpace;
  Rotation2d targetHeading;
  Translation2d targetPosition;

  /** Creates a new AngleTowardTags. */
  public AngleTowardTags(Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelights = drivetrain.getTrustedLimelights();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (Limelight limelight : Limelight.filterLimelights(limelights)) {
      if (limelight.hasTarget()) {
        targetPoseRobotSpace = limelight.getTargetPoseRobotSpace().toPose2d();
        break;
      }
    }
    targetHeading = targetPoseRobotSpace.getRotation();
    targetPosition = targetPoseRobotSpace.getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    point.withModuleDirection(targetHeading);
    drivetrain.applyRequest(() -> point);

    for (Limelight limelight : Limelight.filterLimelights(limelights)) {
      if (limelight.hasTarget()) {
        targetPoseRobotSpace = limelight.getTargetPoseRobotSpace().toPose2d();
        break;
      }
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
