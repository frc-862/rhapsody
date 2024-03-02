// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;



public class AlignToTag extends Command {
  
  public Pose2d target;
  public Swerve drivetrain;
  public MoveToPose moveToPose;
  public PointAtTag pointAtTag;

  private PathPlannerPath path;
  private boolean aligning;
  /** Creates a new AlignToTag. */
  public AlignToTag(Pose2d target, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aligning = false;
    path = PathPlannerPath.fromPathFile("Amp autoalign");
    moveToPose = new MoveToPose(target, drivetrain);
    moveToPose.initialize();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!aligning){
      moveToPose.execute();
      if(moveToPose.isFinished()){
        aligning = true;
      }
    } else {
      AutoBuilder.followPath(path);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getPose().get().getTranslation().getX() == target.getTranslation().getX() 
    && drivetrain.getPose().get().getTranslation().getY() == target.getTranslation().getY();
  }
}
