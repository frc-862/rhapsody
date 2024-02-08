package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SmartCollect extends Command {

	private Indexer indexer;
	
	// Declares collector
	private Collector collector;
	private DoubleSupplier powerSupplier;

	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 */
	public SmartCollect(DoubleSupplier powerSupplier, Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;
		this.powerSupplier = powerSupplier;

		addRequirements(collector, indexer);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		// if getentrybeambreakstate and the exit one are false then run both collector and indxer
		if (!indexer.getEntryBeamBreakState() && !indexer.getExitBeamBreakState()) {
			collector.setPower(powerSupplier.getAsDouble());
			indexer.setPower(powerSupplier.getAsDouble());

		}
		// if getenrtrybeambreakstate true and not the other one then slow down the collector and indexer

		if (indexer.getEntryBeamBreakState() && !indexer.getExitBeamBreakState()) {
			collector.setPower(0.5*powerSupplier.getAsDouble());
			indexer.setPower(0.5*powerSupplier.getAsDouble());
		}
		// if both of them are true then stop both collector and indexer
		if (indexer.getEntryBeamBreakState() && indexer.getExitBeamBreakState()) {
			collector.setPower(0);
			indexer.setPower(0);
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
