package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collect extends Command {

	// Declares pivot
	private Pivot pivot;
	private DoubleSupplier powerSupplier;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param pivot subsystem
	 */
	public Collect(DoubleSupplier powerSupplier, Pivot pivot) {
		this.pivot = pivot;
		this.powerSupplier = powerSupplier;

		addRequirements(pivot);
	}

	@Override
	public void initialize() {
		pivot.setPower(powerSupplier.getAsDouble());
		LightningShuffleboard.setDoubleSupplier("Pivot", "Power", () -> powerSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		pivot.setPower(powerSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		pivot.setPower(0d);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
