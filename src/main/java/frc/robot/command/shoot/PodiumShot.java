package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PodiumShot extends Command {

	private final Pivot pivot;
	private final Flywheel flywheel;

	/**
	 * Creates a new PodiumShot.
	 * @param pivot
	 * @param flywheel
	 */
	public PodiumShot(Pivot pivot, Flywheel flywheel) {
		this.pivot = pivot;
		this.flywheel = flywheel;

		addRequirements(pivot, flywheel);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		pivot.setTargetAngle(CandConstants.PODIUM_ANGLE + pivot.getBias());
		flywheel.setAllMotorsRPM(CandConstants.PODIUM_RPM + flywheel.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		flywheel.coast(true);
	}
}
