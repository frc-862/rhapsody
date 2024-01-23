package frc.robot.command;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Collect extends Command {

	// Declares collector
	private Collector collector;
	private DoubleSupplier powerSupplier;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 */
	public Collect(DoubleSupplier powerSupplier, Collector collector) {
		this.collector = collector;
		this.powerSupplier = powerSupplier;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPower(powerSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		collector.setPower(powerSupplier.getAsDouble());
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
