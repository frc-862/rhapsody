package frc.robot.command;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.vision.Limelight;

public class UpdateOrientation extends Command {

	private Limelight stopMe;
	private Pigeon2 pigeon;
	private double drivetrainYaw;
	private double drivetrainYawRate;

	/**
	 * Creates a new UpdateOrientation.
	 * 
	 * @param limelights
	 * @param drivetrain
	 */
	public UpdateOrientation(Limelights limelights, Swerve drivetrain) {
		this.stopMe = limelights.getStopMe();
		pigeon = drivetrain.getPigeon2();

		addRequirements(limelights);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drivetrainYawRate = pigeon.getRate();
		if (!(drivetrainYawRate > 90)) {
			drivetrainYaw = pigeon.getAngle();
			stopMe.setRobotOrientation((drivetrainYaw) % 360, drivetrainYawRate, 0d, 0d, 0d, 0d);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
