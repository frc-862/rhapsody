package frc.robot.command;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class Index extends Command {

	private DoubleSupplier power;
	private Indexer indexer;

	/**
	 * Creates a new Index.
	 * @param power supplier for motor power
	 * @param indexer subsystem
	 */
	public Index(DoubleSupplier power, Indexer indexer) {
		this.power = power;
		this.indexer = indexer;

		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		indexer.setPower(power.getAsDouble());
	}

	@Override
	public void execute() {
		indexer.setPower(power.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}