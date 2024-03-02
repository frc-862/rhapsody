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
		flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		// Checks if autonomous and if the pivot and flywheel are on target then shoots
		if (pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
			indexer.indexUp();
		}

		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
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