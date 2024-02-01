package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.util.Units;

public class pathfinding extends Command {
	
	Swerve drivetrain;
	Command pathfindingCommand;
	SwerveRequest brake;

	public pathfinding(Swerve drivetrain, SwerveRequest brake) {
		
		this.drivetrain = drivetrain;
		this.brake = brake;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		pathfindingCommand = AutoBuilder.pathfindToPose(
			AutonomousConstants.TARGET_POSE,
			AutonomousConstants.PATH_CONSTRAINTS,
			0.0, // Goal end velocity in meters/sec
			0.0); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
			System.out.println("Created pathfinding");
		System.out.println("Pathfinding Init");
		pathfindingCommand.initialize();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println("Excuting pathfinder");
		pathfindingCommand.execute();		
		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		pathfindingCommand.end(interrupted);
		drivetrain.applyRequest(() -> brake);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pathfindingCommand.isFinished();
	}
}
// Load the path we want to pathfind to and follow 
//PathPlannerPath path = PathPlannerPath.fromPathFile("1MeterSquareLL"); 

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path. 
//PathConstraints constraints = new PathConstraints(
	//3.0, 4.0,
	//Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands 
//Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
	//path,
	//constraints,
	//3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. 
//);
