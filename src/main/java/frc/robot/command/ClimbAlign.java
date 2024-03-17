// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Swerve;

public class ClimbAlign extends Command {
  private Swerve drivetrain;
  private PathFindToAuton pathFind;

  /** Creates a new ClimbAlign. */
  public ClimbAlign(Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initCommand().schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathFind.cancel();
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFind.isFinished();
  }

  /**
   * @return start poses of climb autons
   * <li> 0 - center
   * <li> 1 - high
   * <li> 2 - low
   */
  private Pose2d[] getPoses(){
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return new Pose2d[] {
        ClimbConstants.PATHFIND_CENTER_STAGE_START_POSE_BLUE,
        ClimbConstants.PATHFIND_HIGH_STAGE_START_POSE_BLUE,
        ClimbConstants.PATHFIND_LOW_STAGE_START_POSE_BLUE
      };
    }
    return new Pose2d[] {
      ClimbConstants.PATHFIND_CENTER_STAGE_START_POSE_RED,
      ClimbConstants.PATHFIND_HIGH_STAGE_START_POSE_RED,
      ClimbConstants.PATHFIND_LOW_STAGE_START_POSE_RED
    };
  }

  /**
   * Determine get differences between robotPose and auton startposes
   * @return differences
   * <li> 0 - diffCenter
   * <li> 1 - diffHigh
   * <li> 2 - diffLow
   */
  private double[] getDiffs() {

    double diffCenterX = Math.abs(drivetrain.getPose().getTranslation().getX() - getPoses()[0].getTranslation().getX());
    double diffCenterY = Math.abs(drivetrain.getPose().getTranslation().getY() - getPoses()[0].getTranslation().getY());
    double diffCenter = Math.hypot(diffCenterX, diffCenterY);

    double diffHighX = Math.abs(drivetrain.getPose().getTranslation().getX() - getPoses()[1].getTranslation().getX());
    double diffHighY = Math.abs(drivetrain.getPose().getTranslation().getY() - getPoses()[1].getTranslation().getY());
    double diffHigh = Math.hypot(diffHighX, diffHighY);

    double diffLowX = Math.abs(drivetrain.getPose().getTranslation().getX() - getPoses()[2].getTranslation().getX());
    double diffLowY = Math.abs(drivetrain.getPose().getTranslation().getY() - getPoses()[2].getTranslation().getY());
    double diffLow = Math.hypot(diffLowX, diffLowY);

    return new double[] {diffCenter, diffHigh, diffLow};
  }

  /**
   * Create the pathfind command based on the shortest distance
   * @return command
   */
  public Command initCommand(){
    if (getDiffs()[0] < getDiffs()[1] && getDiffs()[0] < getDiffs()[2]) {
      pathFind = new PathFindToAuton(PathPlannerPath.fromPathFile("PathFind-CENTER-STAGE"), drivetrain);
    } else 
    if (getDiffs()[1] < getDiffs()[0] && getDiffs()[1] < getDiffs()[2]) {
      pathFind = new PathFindToAuton(PathPlannerPath.fromPathFile("PathFind-HIGH-STAGE"), drivetrain);
    } else {
      pathFind = new PathFindToAuton(PathPlannerPath.fromPathFile("PathFind-LOW-STAGE"), drivetrain);
    }
    return pathFind;
  }
}