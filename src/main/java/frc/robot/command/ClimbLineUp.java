package frc.robot.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Swerve;

public class ClimbLineUp extends SequentialCommandGroup {

	public ClimbLineUp(Swerve drivetrain) {
    	addCommands(new MoveToPose(ClimbConstants.BACK_CLIMB, drivetrain));
  	}
}
