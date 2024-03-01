package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collect extends Command {

	private DoubleSupplier powerSupplier;
	private Collector collector;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 */
	public Collect(DoubleSupplier powerSupplier, Collector collector) {
		this.powerSupplier = powerSupplier;
		this.collector = collector;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPower(powerSupplier.getAsDouble());
		System.out.println("Collect.init");
		LightningShuffleboard.setDoubleSupplier("Collector", "Power", () -> powerSupplier.getAsDouble());
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
