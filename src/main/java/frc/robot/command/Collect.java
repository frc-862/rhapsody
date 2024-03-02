package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collect extends Command {

	// Declares collector
	private DoubleSupplier powerSupplier;
	private Collector collector;
	private Indexer indexer;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 * @param indexer subsystem
	 */
	public Collect(DoubleSupplier powerSupplier, Collector collector, Indexer indexer) {
		this.powerSupplier = powerSupplier;
		this.collector = collector;
		this.indexer = indexer;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPower(powerSupplier.getAsDouble());
		LightningShuffleboard.setDoubleSupplier("Collector", "Power", powerSupplier);
	}

	@Override
	public void execute() {
		collector.setPower(powerSupplier.getAsDouble());
		if (powerSupplier.getAsDouble() > 0d) {
			indexer.setPower(1d);
		} else {
			indexer.stop();
		}
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
