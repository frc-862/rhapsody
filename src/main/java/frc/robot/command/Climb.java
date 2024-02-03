package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMBER_STATES;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
public class Climb extends Command {
	private Climber climber;
	boolean hasTipped = false;
	private TipDetection tipDetection;
	BooleanSupplier abort;

	/**
	 * Creates a new Climb.
	 * 
	 * @param climber subsystem
	 * @param drivetrain subsystem
	 */
	public Climb(Climber climber, Swerve drivetrain, BooleanSupplier abort) {
		this.climber = climber;
		this.tipDetection = new TipDetection(drivetrain);
		this.abort = abort;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		// extend arm is preperation for climbing
		if (climber.getState() == CLIMBER_STATES.STOW){
			climber.setSetpoint(ClimbConstants.MAX_HEIGHT);
			climber.hasPressed();
		}



	}

	@Override
	public void execute() {

		switch (climber.getState()) {
			case STOW:
				// check if robot is tipped when arm is extended more than half max height (2 ft probably)
				if (climber.getHeightL() > ClimbConstants.MAX_HEIGHT/2 &&
					climber.getHeightR() > ClimbConstants.MAX_HEIGHT/2 &&
					tipDetection.isTipped()) {
					climber.setSetpoint(0d);
					climber.setHasTipped(true);
					
				}
				break;
			case CLIMBED:
				// re-extend arm slowly to return to ground
				if (ClimbConstants.MAX_HEIGHT - climber.getHeightR() >= ClimbConstants.CLIMB_EXTENSION_TOLERANCE){
					climber.setPowerR(ClimbConstants.CLIMB_RETURN_TO_GROUND_MAX_POWER);					
				} 
				else if (ClimbConstants.MAX_HEIGHT - climber.getHeightR() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE){
					climber.setPowerR(0d);
					climber.setHasGroundedR(true);
				}

				if (ClimbConstants.MAX_HEIGHT - climber.getHeightL() >= ClimbConstants.CLIMB_EXTENSION_TOLERANCE){
					climber.setPowerL(ClimbConstants.CLIMB_RETURN_TO_GROUND_MAX_POWER);					
				} 
				else if (ClimbConstants.MAX_HEIGHT - climber.getHeightL() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE){
					climber.setPowerL(0d);
					climber.setHasGroundedL(true);
				}
				break;
			case GROUNDED:
				// retract arm after returning to ground and robot moved away from chain
				climber.setSetpoint(0d);
				break;
		}
	}

	@Override
	public void end(boolean interrupted){
		abort();
	}

	@Override
	public boolean isFinished() {
		return abort.getAsBoolean;
	}

	public void abort(){
		Climber.setState(CLIMBER_STATES.CLIMBED);
		climber.stop();
	}
}
