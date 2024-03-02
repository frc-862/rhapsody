package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Flywheel;

public class SourceCollect extends Command {

	private final Pivot pivot;
	private final Flywheel flywheel;

	/**
	 * Creates a new SourceCollect.
	 * @param pivot subsystem
	 * @param flywheel subsystem
	 */
	public SourceCollect(Pivot pivot, Flywheel flywheel) {
		this.pivot = pivot;
		this.flywheel = flywheel;

		addRequirements(flywheel);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
		// pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
		// pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		// pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
