package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PodiumShot extends Command {
	
	private final Flywheel flywheel;
	private final Pivot pivot;
	
	/** Creates a new PodiumShot.
	 * @param flywheel
	 * @param pivot
	 */
	public PodiumShot(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(pivot, flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.PODIUM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.PODIUM_ANGLE + pivot.getBias());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.coast();
		pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
		//TODO add LED state
	}
}