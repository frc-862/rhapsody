package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class stopDrive extends Command {
	
	private Swerve drivetrain;

	public stopDrive(Swerve drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("AUTO - Stop Drivetrain INIT");
		drivetrain.applyPercentRequestField(() -> 0d, () -> 0d, () -> 0d);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// drivetrain.applyPercentRequestField(() -> 0d, () -> 0d, () -> 0d);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("AUTO - Stop Drivetrain END");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
