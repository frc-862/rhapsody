package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Flywheel;

public class AmpShot extends Command {

	private final Pivot pivot;
	private final Flywheel flywheel;

	/**
	 * Creates a new AmpShot
	 * @param pivot subsystem
	 * @param flywheel subsystem
	 */
	public AmpShot(Pivot pivot, Flywheel flywheel) {
		this.pivot = pivot;
		this.flywheel = flywheel;

		addRequirements(pivot, flywheel);
	}

	@Override
	public void initialize() {
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
	}

	@Override
	public void execute() {
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
		flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
	}

	@Override
	public void end(boolean interrupted) {
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		flywheel.coast(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
