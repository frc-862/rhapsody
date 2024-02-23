package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AmpShot extends Command {

	private Pivot pivot;
	private Flywheel flywheel;
	// private Indexer indexer;
	private boolean isAutonomous;
	private boolean shot = false;
	private double shotTime = 0;

	//TODO temp
	private double topRPM = CandConstants.AMP_TOP_RPM;
	private double bottomRPM = CandConstants.AMP_BOTTOM_RPM;
	private double pivotAngle = CandConstants.AMP_ANGLE;

	/**
	 * Creates a new AmpShot
	 * @param pivot subsystem
	 * @param flywheel subsystem
	 * @param indexer subsystem
	 * @param isAutonomous boolean if robot is in autonomous
	 */
	public AmpShot(Flywheel flywheel, Pivot pivot, boolean isAutonomous) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		// this.indexer = indexer;
		this.isAutonomous = isAutonomous;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {
		// flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		// flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		// pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		// Checks if autonomous and if the pivot and flywheel are on target then shoots
		// if (isAutonomous && pivot.onTarget() && flywheel.allMotorsOnTarget()) {
		// 	shot = true;
		// 	shotTime = Timer.getFPGATimestamp();
		// 	indexer.indexUp();
		// }

		pivotAngle = LightningShuffleboard.getDouble("Amp", "Pivot target", CandConstants.AMP_ANGLE + pivot.getBias());
		topRPM = LightningShuffleboard.getDouble("Amp", "Top RPM target", CandConstants.AMP_TOP_RPM + flywheel.getBias());
		bottomRPM = LightningShuffleboard.getDouble("Amp", "Bottom RPM target", CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		
		// pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		// flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		// flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());

		pivot.setTargetAngle(pivotAngle);
		flywheel.setTopMoterRPM(topRPM);
		flywheel.setBottomMoterRPM(bottomRPM);
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		// indexer.stop();
	}

	@Override
	public boolean isFinished() {
		// if (isAutonomous) {
		// 	return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT;
		// }
		return false;
	}
}
