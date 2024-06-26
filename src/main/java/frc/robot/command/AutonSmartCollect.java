package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class AutonSmartCollect extends Command {

    private DoubleSupplier collectorPower;
    private DoubleSupplier indexerPower;
    private Collector collector;
    private Indexer indexer;

    /**
     * SmartCollect to control collector and indexer using beambreaks
     *
     * @param collectorPower power to apply to collector
     * @param indexerPower   power to apply to indexer
     * @param collector      subsystem
     * @param indexer        subsystem
     */
    public AutonSmartCollect(DoubleSupplier collectorPower, DoubleSupplier indexerPower, Collector collector, Indexer indexer) {
        this.collectorPower = collectorPower;
        this.indexerPower = indexerPower;

        this.collector = collector;
        this.indexer = indexer;

        addRequirements(collector, indexer);
    }

    @Override
    public void initialize() {
        collector.setPower(collectorPower.getAsDouble());
        indexer.setPower(indexerPower.getAsDouble());

        System.out.println("COLLECT - Auton Smart Collect INIT");
    }

    @Override
    public void execute() {
        if (indexer.getPieceState() == PieceState.IN_COLLECT) {
            indexer.setPower(indexerPower.getAsDouble() / 2);
            // collector.setPower(collectorPower.getAsDouble() / 2);
        } else {
            collector.setPower(collectorPower.getAsDouble());
            indexer.setPower(indexerPower.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
        indexer.stop();

        System.out.println("COLLECT - Auton Smart Collect END");
    }

    @Override
    public boolean isFinished() {
        return indexer.getPieceState() == PieceState.IN_PIVOT;
    }
}
