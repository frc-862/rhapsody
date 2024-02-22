package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class SmartCollect extends Command {

	private Collector collector;
	private Indexer indexer;
	private DoubleSupplier collectorPower;
	private DoubleSupplier indexerPower;

	/**
	 * SmartCollect to control collector and indexer using beambreaks
	 * @param collectorPower power to apply to collector
	 * @param indexerPower power to apply to indexer
	 * @param collector subsystem
	 * @param indexer subsystem
	 */
	public SmartCollect(DoubleSupplier collectorPower, DoubleSupplier indexerPower, Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;
		this.collectorPower = collectorPower;
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
