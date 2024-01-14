package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Collect extends Command {

	// Declares collector
	private Collector collector;
	private DoubleSupplier powerSupplier;

	public Collect(DoubleSupplier powerSupplier, Collector collector) {
		// Sets collector from parameter
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
