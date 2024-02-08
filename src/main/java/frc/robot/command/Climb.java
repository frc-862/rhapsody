package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class Climb extends Command {
	private final Climber climber;
	private final Swerve drivetrain;

	LEDs leds;

	/**
	 * Creates a new Climb.
	 * @param climber subsystem
	 * @param drivetrain subsystem
	 * @param leds to
	 */
	public Climb(Climber climber, Swerve drivetrain, LEDs leds) {
		this.climber = climber;
		this.drivetrain = drivetrain;
		this.leds = leds;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		// extend arm is preperation for climbing
			climber.setSetpoint(ClimbConstants.MAX_HEIGHT);
	}

	@Override
	public void execute() {
		switch (climber.getState()) {
			case STOW:
				// check if robot is tipped when arm is extended more than half max height (2 ft probably)
				if (climber.getHeightL() > ClimbConstants.MAX_HEIGHT / 2
						&& climber.getHeightR() > ClimbConstants.MAX_HEIGHT / 2
						&& drivetrain.isTipped()) {
					climber.setSetpoint(ClimbConstants.CLIMB_PID_SETPOINT_RETRACTED);
					climber.setHasClimbed(true);
				}
				break;
			case CLIMBED:
				// re-extend arm slowly to return to ground
				if (ClimbConstants.MAX_HEIGHT
						- climber.getHeightR() >= ClimbConstants.CLIMB_EXTENSION_TOLERANCE) {
					climber.setPowerR(ClimbConstants.CLIMB_RETURN_TO_GROUND_MAX_POWER);
				} else if (ClimbConstants.MAX_HEIGHT
						- climber.getHeightR() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE) {
					climber.setPowerR(0d);
					climber.setHasGroundedR(true);
				}

				if (ClimbConstants.MAX_HEIGHT
						- climber.getHeightL() >= ClimbConstants.CLIMB_EXTENSION_TOLERANCE) {
					climber.setPowerL(ClimbConstants.CLIMB_RETURN_TO_GROUND_MAX_POWER);
				} else if (ClimbConstants.MAX_HEIGHT
						- climber.getHeightL() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE) {
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
		climber.setSetpoint(0d);
		leds.enableState(LED_STATES.FINISHED_CLIMB).withTimeout(2).schedule();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
