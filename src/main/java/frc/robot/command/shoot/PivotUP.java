package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PivotUP extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;

	/**
	 * Creates a new SourceCollect.
	 * @param flywheel subsystem
	 * @param pivot subsystem
	 */
	public PivotUP(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(pivot);
	}

	@Override
	public void initialize() {
		// flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
		pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void execute() {
		// flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
		pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void end(boolean interrupted) {
		// flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
