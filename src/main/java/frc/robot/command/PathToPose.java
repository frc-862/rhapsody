// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class PathToPose extends Command {

  private Swerve drivetrain;
  private XboxControllerFilter controller; // Driver Controller
  private Pose2d pathfindingPose;
  private AutoBuilder autoBuilder;
  Command pathFindCommand;
  
  /*
   * Pathfinds to a specific pose given
   * @param pathfindingPose The pose to pathfind to
   * @param drivetrain The drivetrain subsystem
   * @param controller The driver controller
   */
  public PathToPose(Pose2d pathfindingPose, Swerve drivetrain, XboxControllerFilter controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.pathfindingPose = pathfindingPose;

    autoBuilder = new AutoBuilder();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFindCommand = autoBuilder.pathfindToPose(
      pathfindingPose, AutonomousConstants.PATH_CONSTRAINTS
    );
    pathFindCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getYButton()) {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pathFindCommand.cancel();
    }
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
