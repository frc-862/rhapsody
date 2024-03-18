package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class PathToPose extends Command {

    private XboxControllerFilter controller = RobotContainer.driver;
    private Pose2d pathfindingPose;
    Command pathFindCommand;

    /*
     * Pathfinds to a specific pose given
     * @param pathfindingPose The pose to pathfind to
     * @param drivetrain The drivetrain subsystem
     * @param controller The driver controller
     */
    public PathToPose(Pose2d pathfindingPose) {
        this.pathfindingPose = pathfindingPose;
    }

    @Override
    public void initialize() {
        pathFindCommand = AutoBuilder.pathfindToPose(
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
