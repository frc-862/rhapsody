package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class CandC3 extends Command {
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Indexer indexer;
	private boolean shot = false;
	private double shotTime = 0;
	
	/** Creates a new PodiumShot.
	 * @param flywheel
	 * @param pivot
	 * @param indexer
	 */
	public CandC3(Flywheel flywheel, Pivot pivot, Indexer indexer) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.indexer = indexer;
	
		addRequirements(pivot, flywheel, indexer);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.PODIUM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.PODIUM_ANGLE + pivot.getBias());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			indexer.setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
			shot = true;
			shotTime = Timer.getFPGATimestamp();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.coast();
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		indexer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT; 
	}
}
