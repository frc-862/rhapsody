package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class preAim extends Command {

	Flywheel flywheel; 
	Pivot pivot;
	Swerve drivetrain;

	public preAim(Flywheel flywheel, Pivot pivot, Swerve drivetrain) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.drivetrain = drivetrain;

		addRequirements(flywheel, pivot);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("AUTO - PRE AIM START");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double distance = drivetrain.distanceToSpeaker();
		pivot.setTargetAngle(calculateTargetAngle(distance));
		flywheel.setAllMotorsRPM(calculateTargetRPM(distance));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("AUTO - PRE AIM END");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	/**
	 * Calculate Pivot Target angle (in degrees)
	 * 
	 * @param distance from the speaker
	 * @return Angle to set pivot to
	 */
	public double calculateTargetAngle(double distance) {
		return ShooterConstants.ANGLE_MAP.get(distance);
	}

	/**
	 * Calculate Flywheel Target RPM (in RPM)
	 * 
	 * @param distance from the speaker
	 * @return RPM to set the Flywheels
	 */
	public double calculateTargetRPM(double distance) {
		return ShooterConstants.SPEED_MAP.get(distance);
	}
}
