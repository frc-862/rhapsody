// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.PathFindingConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class PathFindToAuton extends Command {

  private Swerve drivetrain;
  private XboxControllerFilter controller; // Driver Controller
  private AutoBuilder autoBuilder;
  private Command pathFindCommand;
  private PathPlannerPath autonPath;
  
  /*
   * Pathfinds to a specific pose given
   * @param pathfindingPose The pose to pathfind to
   * @param drivetrain The drivetrain subsystem
   * @param controller The driver controller
   */
  public PathFindToAuton(PathPlannerPath autonPath, Swerve drivetrain) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autonPath = autonPath;
    autoBuilder = new AutoBuilder();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFindCommand = autoBuilder.pathfindThenFollowPath(
      autonPath, AutonomousConstants.PATH_CONSTRAINTS);
    pathFindCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathFindCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathFindCommand.isFinished()) {
      return true;
    }
    return false;
  }
}
