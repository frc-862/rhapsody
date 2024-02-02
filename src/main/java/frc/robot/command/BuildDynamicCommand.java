// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;

public class BuildDynamicCommand extends Command {
  Swerve drivetrain;
  Command buildDynamicComamand;

  public BuildDynamicCommand(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      buildDynamicComamand = AutoBuilder.pathfindToPose(
			AutonomousConstants.TARGET_POSE,
			AutonomousConstants.PATH_CONSTRAINTS,
			0.0, // Goal end velocity in meters/sec
			0.0); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
