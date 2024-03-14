// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.PathFindToAuton;
import frc.robot.subsystems.Swerve;

public class AmpAlign extends Command {
  /** Creates a new AmpAlign. */

  Swerve drivetrain;
  PathPlannerPath path;
  PathFindToAuton pathCommand;

  public AmpAlign(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      path = PathPlannerPath.fromPathFile("PathFind-AMP");
      PathFindToAuton pathCommand = new PathFindToAuton(path, drivetrain);
      pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
