package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class PointBlankShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;
	private boolean isAutonomous;
	private boolean shot = false;
	private double shotTime = 0;

	/**
	 * Creates a new PointBlankShot.
	 * @param flywheel
	 * @param pivot
	 * @param isAutonomous
	 */
	public PointBlankShot(Flywheel flywheel, Pivot pivot, boolean isAutonomous) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.isAutonomous = isAutonomous;

		addRequirements(pivot, flywheel);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.POINT_BLANK_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.POINT_BLANK_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		if(pivot.onTarget() && flywheel.allMotorsOnTarget()) {
			shot = true;
			shotTime = Timer.getFPGATimestamp();
		}
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		//TODO add LED state
	}

	@Override
	public boolean isFinished() {
		if(isAutonomous){
			return shot && Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT;
		}
		return false;
	}
}