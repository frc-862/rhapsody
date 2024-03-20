// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class PathFindToAuton extends Command {

    private Swerve drivetrain;
    private XboxControllerFilter controller; // Driver Controller
    private AutoBuilder autoBuilder;
    private Command pathFindCommand;
    private PathPlannerPath autonPath;

    /**
     * Pathfinds to a specific pose given
     *
     * @param autonPath
     * @param drivetrain
     * @param controller
     */
    public PathFindToAuton(PathPlannerPath autonPath, Swerve drivetrain) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.autonPath = autonPath;
        autoBuilder = new AutoBuilder();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathFindCommand = autoBuilder.pathfindThenFollowPath(
                autonPath, AutonomousConstants.PATH_CONSTRAINTS);
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
