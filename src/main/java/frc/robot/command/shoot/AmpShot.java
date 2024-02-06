package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class AmpShot extends Command {

	private Pivot pivot;
	private Flywheel flywheel;

	/**
	 * Creates a new AmpShot.
	 * @param pivot
	 * @param flywheel
	 */
	public AmpShot(Pivot pivot, Flywheel flywheel) {
		this.flywheel = flywheel;
		this.pivot = pivot;
	
		addRequirements(pivot, flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.AMP_RPM);
		pivot.setTargetAngle(CandConstants.AMP_ANGLE);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.coast();
		pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
		//TODO add LED state
	}
}