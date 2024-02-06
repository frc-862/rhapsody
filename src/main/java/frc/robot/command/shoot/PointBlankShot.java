package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PointBlankShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;

	/**
	 * Creates a new PointBlankShot.
	 * @param flywheel 
	 * @param pivot
	 */
	public PointBlankShot(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(flywheel);
		addRequirements(pivot);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM);
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.coast();
		pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
		//TODO add LED state
	}
}