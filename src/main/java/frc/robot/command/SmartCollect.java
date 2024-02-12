package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class SmartCollect extends Command {

	private Indexer indexer;
	private Collector collector;
	private DoubleSupplier powerSupplier;

	/**
	 * Creates a new SmartCollect.
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
	public void initialize() {}

	@Override
	public void execute() {
		switch(indexer.getPieceState()) {
			case NONE: 
				collector.setPower(powerSupplier.getAsDouble());
				indexer.setPower(powerSupplier.getAsDouble());
				break;

			case IN_PIVOT:
				collector.setPower(0.5*powerSupplier.getAsDouble());
				indexer.setPower(0.5*powerSupplier.getAsDouble());
				break;

			case IN_COLLECT:
				collector.setPower(powerSupplier.getAsDouble());
				indexer.setPower(powerSupplier.getAsDouble());
				break;

			case IN_DEXER:
				collector.setPower(0);
				indexer.setPower(0);
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
