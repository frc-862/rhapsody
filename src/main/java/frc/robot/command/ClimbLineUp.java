package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Swerve;

public class ClimbLineUp extends SequentialCommandGroup {

	public ClimbLineUp(Swerve drivetrain) {
		PathPlannerPath path = PathPlannerPath.fromPathFile("FRONT-Climb-Lineup");
    	addCommands(
			new MoveToPose(ClimbConstants.BACK_CLIMB, drivetrain), 
			new AutoBuilder().followPath(path));
  	}
}
