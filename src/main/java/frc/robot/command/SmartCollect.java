package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SmartCollect extends Command {

	private Collector collector;
	private Indexer indexer;
	private DoubleSupplier collectorPower;
	private DoubleSupplier indexerPower;

	/**
	 * Creates a new SmartCollect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 * @param indexer subsystem
	 */
	public SmartCollect(DoubleSupplier powerSupplier, DoubleSupplier indexerPower, Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;
		this.collectorPower = powerSupplier;
		this.indexerPower = indexerPower;

		addRequirements(collector, indexer);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		switch (indexer.getPieceState()) {
			case NONE, IN_COLLECT: 
				collector.setPower(collectorPower.getAsDouble());
				indexer.setPower(indexerPower.getAsDouble());
				break;

			case IN_PIVOT:
				collector.stop();
				indexer.setPower(0.8*indexerPower.getAsDouble());
				break;
			
			case IN_INDEXER:
				collector.stop();
				indexer.stop();
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
