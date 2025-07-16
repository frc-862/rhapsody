package frc.robot.command.shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PointBlankShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;

	private double powerMult;

	/**
	 * Creates a new PointBlankShot.
	 *
	 * @param pivot    subsystem
	 * @param flywheel subsystem
	 */
	public PointBlankShot(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsDutyCycle(LightningShuffleboard.getDouble("Demo", "Pointblank Power", 25));
		pivot.setTargetAngle(LightningShuffleboard.getDouble("Demo", "Pointblank Angle", CandConstants.POINT_BLANK_ANGLE) + pivot.getBias());
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsDutyCycle(LightningShuffleboard.getDouble("Demo", "Pointblank Power", 25));
		pivot.setTargetAngle(LightningShuffleboard.getDouble("Demo", "Pointblank Angle", CandConstants.POINT_BLANK_ANGLE) + pivot.getBias());
		if(flywheel.allMotorsOnTarget() && pivot.onTarget()) {
			new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
		}
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}