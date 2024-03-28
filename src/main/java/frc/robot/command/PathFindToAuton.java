// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;

public class PathFindToAuton extends Command {

    private Command pathFindCommand;
    private PathPlannerPath autonPath;

    /**
     * Pathfinds to a specific pose given
     *
     * @param autonPath
     * @param drivetrain
     */
    public PathFindToAuton(PathPlannerPath autonPath, Swerve drivetrain) {
        this.autonPath = autonPath;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathFindCommand = AutoBuilder.pathfindThenFollowPath(
                autonPath, AutonomousConstants.PATHFINDING_CONSTRAINTS);
        pathFindCommand.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        pathFindCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return pathFindCommand.isFinished();
    }
}
