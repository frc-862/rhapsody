package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class CandC1 extends Command {

	private final Pivot pivot;
	private final Flywheel flywheel;
	private final Indexer indexer;

	private boolean shot = false;
	private double shotTime = 0;

	private boolean startIndexing = false;

	/**
	 * Creates a new CandC1.
	 * @param pivot subsystem
	 * @param flywheel subsystem
	 * @param indexer subsystem
	 */
	public CandC1(Pivot pivot, Flywheel flywheel, Indexer indexer) {
		this.pivot = pivot;
		this.flywheel = flywheel;
		this.indexer = indexer;

		addRequirements(pivot, flywheel, indexer);
	}

	@Override
	public void initialize() {
		shot = false;
		startIndexing = false;
		pivot.setTargetAngle(CandConstants.C1_ANGLE + pivot.getBias());
		flywheel.setAllMotorsRPM(CandConstants.C1_RPM + flywheel.getBias());
	}

	@Override
	public void execute() {
		// Checks if the pivot and flywheel are on target then shoots
		// also checks whether or not the flywheel's target RPM is greater than 0
		if (pivot.onTarget() && flywheel.allMotorsOnTarget() && (flywheel.getTopMotorRPM() != 0 && flywheel.getBottomMotorRPM() != 0)) {
			startIndexing = true;
		}

		if(startIndexing) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
			indexer.indexUp();
		}

		pivot.setTargetAngle(CandConstants.C1_ANGLE + pivot.getBias());
		flywheel.setAllMotorsRPM(CandConstants.C1_RPM + flywheel.getBias());
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
