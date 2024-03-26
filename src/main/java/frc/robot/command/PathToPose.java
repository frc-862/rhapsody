package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class PathToPose extends Command {

    private XboxControllerFilter controller = RobotContainer.driver;
    private Swerve drivetrain;
    private PIDController headingController = new PIDController(0.1, 0, 0);
    private Pose2d pathfindingPose;
    Command pathFindCommand;

    /*
     * Pathfinds to a specific pose given
     * @param pathfindingPose The pose to pathfind to
     */
    public PathToPose(Pose2d pathfindingPose, Swerve drivetrain) {
        this.pathfindingPose = pathfindingPose;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingController.enableContinuousInput(0, 360);
        headingController.setTolerance(2);
        pathFindCommand = AutoBuilder.pathfindToPose(
                pathfindingPose, AutonomousConstants.PATHFINDING_CONSTRAINTS);
    }

    @Override
    public void execute() {
        if (controller.getYButton()) {
            end(true);
        }

        if (!headingController.atSetpoint()){
            double pidOutput = headingController.calculate(drivetrain.getPose().getRotation().getDegrees(), pathfindingPose.getRotation().getDegrees());
            drivetrain.setField(0, 0, pidOutput);
        } else if (headingController.atSetpoint() && !pathFindCommand.isScheduled()){
            pathFindCommand.schedule();
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
