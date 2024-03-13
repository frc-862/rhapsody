package frc.robot.command.shoot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class AmpShot extends Command {

	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Swerve drivetrain;

	/**
	 * Creates a new AmpShot
	 * 
	 * @param flywheel subsystem
	 * @param pivot    subsystem
	 */
	public AmpShot(Flywheel flywheel, Pivot pivot, Swerve drivetrain) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.drivetrain = drivetrain;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {
		lineUp();
		flywheel.setTopMotorRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMotorRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		flywheel.setTopMotorRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
		flywheel.setBottomMotorRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
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

	public void lineUp() {
		PathPlannerPath path = PathPlannerPath.fromPathFile("PathFind-AMP");
		AutoBuilder.followPath(path).schedule();
	}
}
