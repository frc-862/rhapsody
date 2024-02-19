package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class AmpShot extends Command {

	private Pivot pivot;
	private Flywheel flywheel;
	private boolean isAutonomous;
	private boolean shot = false;
	private double shotTime = 0;

	/**
	 * Creates a new AmpShot.
	 * 
	 * @param pivot
	 * @param flywheel
	 * @param isAutonomous
	 */
	public AmpShot(Flywheel flywheel, Pivot pivot, boolean isAutonomous) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.isAutonomous = isAutonomous;

		addRequirements(pivot, flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setTopMoterRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMoterRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		if (pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.coast();
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		// TODO add LED state
	}

	@Override
	public boolean isFinished() {
		if (isAutonomous) {
			return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT;
		}
		return false;
	}
}
