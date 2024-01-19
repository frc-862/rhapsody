// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

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
import frc.robot.Constants.DrivetrAinConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;
import frc.thunder.vision.Limelight;

public class PointAtTag extends Command {

  private Swerve drivetrain;
  private Limelight[] limelights;
  double targetHeading;
  double pidOutput;

  PIDController headingController = new PIDController(0.1, 0, 0);

  /** Creates a new PointAtTag
   . */
  public PointAtTag(Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelights = drivetrain.getLimelights();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    for (Limelight limelight : Limelight.filterLimelights(limelights)){
      targetHeading = limelight.getTargetX(); 
    }

    LightningShuffleboard.setDouble("PointAtTag", "Target Heading", targetHeading);
    
    headingController.setP(LightningShuffleboard.getDouble("PointAtTag", "P", 0.1));
    headingController.setD(LightningShuffleboard.getDouble("PointAtTag", "D", 0));

    pidOutput = headingController.calculate(0, targetHeading);
    LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);
    SwerveRequest.RobotCentric pointAtTag = new SwerveRequest.RobotCentric();
    drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));

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
