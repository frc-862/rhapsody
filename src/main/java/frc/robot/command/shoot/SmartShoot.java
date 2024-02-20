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
	/** Creates a new SmartShoot. */
	final Flywheel flywheel;
	final Pivot pivot;
	final Swerve drivetrain;
	final Indexer indexer;
	final LEDs leds;
	private boolean hasShot = false;

	public SmartShoot(Flywheel flywheel, Pivot pivot, Swerve drivetrain, Indexer indexer, LEDs leds) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.drivetrain = drivetrain;
		this.indexer = indexer;
		this.leds = leds;

		addRequirements(pivot, flywheel, indexer);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		pivot.setTargetAngle(calculateTargetAngle());
		// flywheel.setAllMotorsRPM(calculateTargetRPM());

		if (onTarget()) {
			indexer.setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
			hasShot = true;
		}
	}

	// Called once the command ends or is interrupted.
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

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false; //TODO add timer + piece passed through indexer
	}

	public boolean onTarget() {
		return flywheel.allMotorsOnTarget() && pivot.onTarget(); //TODO add indexer sensor and drivetrain speed
	}

	/**
	 * calculatePivotTargetAngle
	 * @return Angle to set pivot to
	 */
	public double calculateTargetAngle() {
		return 45d + pivot.getBias(); //TODO Add math from Chris branch or interpolation map
	}

	public double calculateTargetRPM() {
		return 5500 + flywheel.getBias(); // TODO test with interpolation or math 
	}
}