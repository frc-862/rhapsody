package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class SourceCollect extends Command {

	private final Flywheel flywheel;

	// TODO add pivot after Kettering
	/**
	 * Creates a new SourceCollect.
	 * @param flywheel subsystem
	 */
	public SourceCollect(Flywheel flywheel) {
		this.flywheel = flywheel;

		addRequirements( flywheel);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}