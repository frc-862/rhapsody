package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class PointBlankShotAuton extends Command {
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Indexer indexer;
	private boolean shot = false;
	private double shotTime = 0;

	/**
	 * Creates a new PointBlankShot.
	 * @param flywheel subsystem
	 * @param pivot subsystem
	 * @param indexer subsystem
	 */
	public PointBlankShotAuton(Flywheel flywheel, Pivot pivot, Indexer indexer) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.indexer = indexer;

		addRequirements(pivot, flywheel, indexer);
	}

	@Override
	public void initialize() {
		shot = false;
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		// Checks if autonomous and if the pivot and flywheel are on target then shoots
		if(pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
			indexer.indexUp();
		}

		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + pivot.getBias());
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE + flywheel.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT;
	}
}