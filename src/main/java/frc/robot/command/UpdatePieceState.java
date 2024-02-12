package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class UpdatePieceState extends Command {

	private Indexer indexer;
	private Collector collector;

	PieceState currentState;
	boolean collectorBeamBreak;
	boolean indexerEntryBeamBreak;
	boolean indexerExitBeamBreak;

	/**
	 * Creates a new UpdatePieceState.
	 * @param collector subsystem
	 * @param indexer subsystem
	 */
	public UpdatePieceState(Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;

		addRequirements(collector, indexer);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		currentState = indexer.getPieceState();
		collectorBeamBreak = collector.getEntryBeamBreakState();
		indexerEntryBeamBreak = indexer.getEntryBeamBreakState();
		indexerExitBeamBreak = indexer.getExitBeamBreakState();
		
		if (indexerExitBeamBreak) {
			currentState = PieceState.IN_INDEXER;
		} else if (indexerEntryBeamBreak) {
			currentState = PieceState.IN_PIVOT;
		} else if (collectorBeamBreak) {
			currentState = PieceState.IN_COLLECT;
		} else {
			currentState = PieceState.NONE;
		}
		
		indexer.setPieceState(currentState);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
