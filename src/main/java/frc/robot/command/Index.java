package frc.robot.command;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class Index extends Command {
	Indexer indexer;
	DoubleSupplier power;

	/**
	 * Creates a new Index.
	 * @param indexer subsystem
	 * @param power supplier for motor power
	 */
	public Index(Indexer indexer, DoubleSupplier power) {
		this.indexer = indexer;
		this.power = power;

		addRequirements(indexer);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		indexer.setPower(power.getAsDouble());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		indexer.setPower(power.getAsDouble());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		indexer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}