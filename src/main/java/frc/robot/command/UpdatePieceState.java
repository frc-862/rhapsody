package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants.PIECE_STATE;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class UpdatePieceState extends Command {

	private Indexer indexer;
	
	// Declares collector
	private Collector collector;

	PIECE_STATE currState;
	boolean collectorBeamBreak;
	boolean indexerEntryBeamBreak;
	boolean indexerExitBeamBreak;


	/**
	 * Creates a new Collect.
	 * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
	 * @param collector subsystem
	 */
	public UpdatePieceState(Collector collector, Indexer indexer) {
		this.collector = collector;
		this.indexer = indexer;
	

		addRequirements(collector, indexer);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		currState = indexer.getPieceState();
		collectorBeamBreak = collector.getEntryBeamBreakState();
		indexerEntryBeamBreak = indexer.getEntryBeamBreakState();
		indexerExitBeamBreak = indexer.getExitBeamBreakState();
		
		if (indexerExitBeamBreak) {
			currState = PIECE_STATE.IN_DEXER;
		} else if(indexerEntryBeamBreak) {
			currState = PIECE_STATE.IN_PIVOT;
		} else if(collectorBeamBreak) {
			currState = PIECE_STATE.IN_COLLECT;
		} else {
			currState = PIECE_STATE.NONE;
		}
		
		indexer.setPieceState(currState);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
