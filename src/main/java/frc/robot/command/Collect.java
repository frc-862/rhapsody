package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collect extends Command {

	// Declares collector
	private Collector collector;
	private Indexer indexer;
	private DoubleSupplier powerSupplier;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 */
	public Collect(DoubleSupplier powerSupplier, Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;
		this.powerSupplier = powerSupplier;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPower(powerSupplier.getAsDouble());
		LightningShuffleboard.setDoubleSupplier("Collector", "Power", () -> powerSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		collector.setPower(powerSupplier.getAsDouble());
		if(powerSupplier.getAsDouble() > 0d) {
			indexer.setPower(1d);
		} else {
			indexer.setPower(0d);
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
