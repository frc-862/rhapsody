package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class SmartCollect extends Command {

	private DoubleSupplier collectorPower;
	private DoubleSupplier indexerPower;
	private Collector collector;
	private Indexer indexer;
	private Pivot pivot;

	/* Used to prevent indexing if pivot angle is too high */
	private boolean allowIndex;

	/* Used to check if we have already touched the exit beambreak and we need to back down */
	private boolean reversedFromExit = false;

	/**
	 * SmartCollect to control collector and indexer using beambreaks
	 * @param collectorPower power to apply to collector
	 * @param indexerPower power to apply to indexer
	 * @param collector subsystem
	 * @param indexer subsystem
	 * @param pivot subsystem
	 */
	public SmartCollect(DoubleSupplier collectorPower, DoubleSupplier indexerPower, Collector collector, Indexer indexer, Pivot pivot) {
		this.collector = collector;
		this.indexer = indexer;
		this.pivot = pivot;
		this.collectorPower = collectorPower;
		this.indexerPower = indexerPower;

		addRequirements(collector, indexer);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		allowIndex = pivot.getAngle() < Constants.PivotConstants.MAX_INDEX_ANGLE;

		switch (indexer.getPieceState()) {
			case NONE: /* Note not passing any beambreaks */
				reversedFromExit = false;
				collector.setPower(collectorPower.getAsDouble());
				if (allowIndex) {
					indexer.setPower(indexerPower.getAsDouble());
				}
				break;

			case IN_COLLECT: /* Note has passed beambreak past collector */
				if (allowIndex) {
					// Slow down collector to prevent jamming
					collector.setPower(0.65 * collectorPower.getAsDouble());
					indexer.setPower(indexerPower.getAsDouble());
				} else {
					// Stop collecting since pivot is not in right place
					collector.stop();
					indexer.stop();
				}
				break;

			case IN_PIVOT: /* Note has touched entry indexer beambreak */
				collector.stop();
				if (allowIndex && !reversedFromExit) {
					indexer.setPower(0.8 * indexerPower.getAsDouble());
				} else if (reversedFromExit) {
					indexer.stop();
				}
				break;

			case IN_INDEXER: /* Note has touched exit indexer beambreak */
				collector.stop();
				indexer.setPower(-0.25 * indexerPower.getAsDouble());
				reversedFromExit = true;
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
