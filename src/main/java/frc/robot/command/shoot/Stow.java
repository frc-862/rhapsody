package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class Stow extends Command {
	/** Creates a new Stow. */

	private Pivot pivot;
	private Flywheel flywheel;

	public Stow(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(pivot, flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.coast();
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
	}

	@Override
	public boolean isFinished() {
		return pivot.onTarget();
	}
}