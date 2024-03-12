package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PointBlankShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;

	/**
	 * Creates a new PointBlankShot.
	 * 
	 * @param pivot    subsystem
	 * @param flywheel subsystem
	 */
	public PointBlankShot(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + pivot.getBias());
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE + flywheel.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}