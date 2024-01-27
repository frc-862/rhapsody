package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;

public class pathfinding extends Command {
	
	Swerve drivetrain;
	Command pathfindingCommand;
	SwerveRequest brake;

	public pathfinding(Swerve drivetrain, SwerveRequest brake) {
		this.drivetrain = drivetrain;
		this.brake = brake;

		addRequirements(drivetrain);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("Pathfinding Init");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		pathfindingCommand = AutoBuilder.pathfindToPose(
        AutonomousConstants.TARGET_POSE,
        AutonomousConstants.PATH_CONSTRAINTS,
        0.0, // Goal end velocity in meters/sec
        0.0); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
		System.out.println("Pathfinding Execute");
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.applyRequest(() -> brake);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
