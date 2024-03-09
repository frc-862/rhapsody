// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.IndexerConstants.PieceState;

public class CollectAndGo extends Command {

	private final Collector collector;
	private final Flywheel flywheel;
	private final Indexer indexer;

	/** Creates a new CollectAndGo. 
	 * @param collector subsystem
	 * @param flywheel subsystem
	 * @param indexer subsystem
	 */
	public CollectAndGo(Collector collector, Flywheel flywheel, Indexer indexer) {
		this.collector = collector;
		this.flywheel = flywheel;
		this.indexer = indexer;
		addRequirements(collector, flywheel, indexer);
	}

	@Override
	public void initialize() {
		flywheel.stop();
		collector.setPower(CollectorConstants.COLLECTOR_GRABANDGO_POWER);
		indexer.indexUp();
	}

	@Override
	public void execute() {
		flywheel.stop();
		collector.setPower(CollectorConstants.COLLECTOR_GRABANDGO_POWER);
		indexer.indexUp();
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return indexer.getPieceState() != PieceState.NONE;
	}
}
