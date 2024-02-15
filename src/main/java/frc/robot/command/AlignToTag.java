// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.vision.Limelight;
import frc.robot.Constants.VisionConstants;
import frc.robot.command.MoveToPose;
import frc.robot.command.PointAtTag;



public class AlignToTag extends Command {

  public Pose2d target;
  public Swerve drivetrain;
  public MoveToPose moveToPose;
  public PointAtTag pointAtTag;

  private Limelight limelight;
	private XboxController driver;
	private FieldCentric drive;
  private Limelights limelights;
	
	private int limelightPrevPipeline = 0;
  
  /** Creates a new AlignToTag. */
  public AlignToTag(Pose2d target, Swerve drivetrain, SwerveRequest.FieldCentric drive, Limelights limelights, XboxController driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.limelights = limelights;

    limelight = limelights.getStopMe();

		limelightPrevPipeline = limelight.getPipeline();

		limelight.setPipeline(VisionConstants.TAG_PIPELINE);
		
		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      new MoveToPose(target, drivetrain, drive);
    // pointAtTag = new PointAtTag(drivetrain, limelights, driver);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  
    // limelight.getTargetPoseRobotSpace();
    
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
