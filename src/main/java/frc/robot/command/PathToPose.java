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

    @Override
    public void initialize() {
        pathFindCommand = autoBuilder.pathfindToPose(
                pathfindingPose, AutonomousConstants.PATH_CONSTRAINTS);
        pathFindCommand.schedule();
    }

    @Override
    public void execute() {
        if (controller.getYButton()) {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pathFindCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathFindCommand.isFinished();
    }
}
