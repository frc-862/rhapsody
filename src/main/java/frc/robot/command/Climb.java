package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;

public class Climb extends Command {
	private Climber climber;
	boolean hasTipped = false;
	private TipDetection tipDetection;

	/**
	 * Creates a new Climb.
	 * 
	 * @param climber subsystem
	 * @param drivetrain subsystem
	 */
	public Climb(Climber climber, Swerve drivetrain) {
		this.climber = climber;
		this.tipDetection = new TipDetection(drivetrain);

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.setSetpoint(ClimbConstants.MAX_HEIGHT);
	}

	@Override
	public void execute() {
		if (climber.getHeightL() > ClimbConstants.MAX_HEIGHT/2 &&
			climber.getHeightR() > ClimbConstants.MAX_HEIGHT/2 &&
			tipDetection.isTipped()) {
			climber.setSetpoint(0d);
			hasTipped = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		climber.stop();
	}

	@Override
	public boolean isFinished() {
		return (
			hasTipped &&
			climber.getHeightL() < ClimbConstants.MAX_HEIGHT/2 &&
			climber.getHeightR() < ClimbConstants.MAX_HEIGHT/2
		);
	}
}
