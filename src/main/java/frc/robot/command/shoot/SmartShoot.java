package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;

public class SmartShoot extends Command {

	final Flywheel flywheel;
	final Pivot pivot;
	final Swerve drivetrain;
	final Indexer indexer;
	final LEDs leds;

	private boolean hasShot = false;

	/**
	 * SmartShoot to control flywheel, pivot, drivetrain, and indexer
	 * @param flywheel
	 * @param pivot
	 * @param drivetrain
	 * @param indexer
	 * @param leds
	 */
	public SmartShoot(Flywheel flywheel, Pivot pivot, Swerve drivetrain, Indexer indexer, LEDs leds) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.drivetrain = drivetrain;
		this.indexer = indexer;
		this.leds = leds;

		addRequirements(pivot, flywheel, indexer);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		pivot.setTargetAngle(calculateTargetAngle());
		flywheel.setAllMotorsRPM(calculateTargetRPM());

		if (onTarget()) {
			indexer.setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
			hasShot = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (hasShot) {
			flywheel.coast();
			pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
			indexer.stop();
			new TimedCommand(RobotContainer.hapticDriverCommand(), 1d).schedule();
			leds.enableState(LED_STATES.SHOT).withTimeout(2).schedule();
		} else {
			flywheel.coast();
		}
	}

	@Override
	public boolean isFinished() {
		return false; // TODO add timer + piece passed through indexer
	}

	public boolean onTarget() {
		return flywheel.allMotorsOnTarget() && pivot.onTarget(); // TODO add indexer sensor and drivetrain speed
	}

	/**
	 * calculatePivotTargetAngle
	 * @return Angle to set pivot to
	 */
	public double calculateTargetAngle() {
		return 45d + pivot.getBias(); // TODO Add math from Chris branch or interpolation map
	}

	public double calculateTargetRPM() {
		return 5500 + flywheel.getBias(); // TODO test with interpolation or math
	}
}