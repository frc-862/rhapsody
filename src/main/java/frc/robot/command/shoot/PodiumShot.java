package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PodiumShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;

	/**
	 * Creates a new PodiumShot.
	 * @param flywheel subsystem
	 * @param pivot subsystem
	 */
	public PodiumShot(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(CandConstants.PODIUM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.PODIUM_ANGLE + pivot.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAnlge());
	}
}
