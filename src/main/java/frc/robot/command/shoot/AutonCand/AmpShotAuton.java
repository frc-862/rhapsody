package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class AmpShotAuton extends Command {

	private Pivot pivot;
	private Flywheel flywheel;
	private Indexer indexer;

	private boolean shot = false;
	private double shotTime = 0;
	
	private boolean startIndexing = false;

	/**
	 * Creates a new AmpShot
	 * @param pivot subsystem
	 * @param flywheel subsystem
	 * @param indexer subsystem
	 */
	public AmpShotAuton(Flywheel flywheel, Pivot pivot, Indexer indexer) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.indexer = indexer;

		addRequirements(flywheel, pivot, indexer);
	}

	@Override
	public void initialize() {
		shot = false;
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + flywheel.getBias());
	}

	@Override
	public void execute() {
		// Checks if the pivot and flywheel are on target then shoots
		if (pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			startIndexing = true;
		}

		if(startIndexing) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
			indexer.indexUp();
		}

		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + flywheel.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		flywheel.coast(true);
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT;
	}
}
