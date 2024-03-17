package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class stopDrive extends Command {

	private Swerve drivetrain;

	public stopDrive(Swerve drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		System.out.println("AUTO - Stop Drivetrain INIT");
		drivetrain.applyPercentRequestField(() -> 0d, () -> 0d, () -> 0d);
	}

	@Override
	public void execute() {
		drivetrain.applyPercentRequestField(() -> 0d, () -> 0d, () -> 0d);
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("AUTO - Stop Drivetrain END");
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
