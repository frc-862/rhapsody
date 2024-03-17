package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Pivot;

public class PivotUP extends Command {

	private final Pivot pivot;

	/**
	 * Creates a new Pivot up Command
	 * Puts the pivot to 90 degrees
	 * @param pivot subsystem
	 */
	public PivotUP(Pivot pivot) {
		this.pivot = pivot;

		addRequirements(pivot);
	}

	@Override
	public void initialize() {
		pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void execute() {
		pivot.setTargetAngle(CandConstants.SOURCE_ANGLE);
	}

	@Override
	public void end(boolean interrupted) {
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
