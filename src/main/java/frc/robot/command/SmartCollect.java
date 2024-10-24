package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;

public class SmartCollect extends Command {

    private DoubleSupplier collectorPower;
    private DoubleSupplier indexerPower;
    private Collector collector;
    private Indexer indexer;
    private Pivot pivot;
    private Flywheel flywheel;

    /* Used to prevent indexing if pivot angle is too high */
    private boolean allowIndex;

    /*
     * Used to check if we have already touched the exit beambreak and we need to
     * back down
     */
    private boolean reversedFromExit = false;

    /**
     * SmartCollect to control collector and indexer using beambreaks
     *
     * @param collectorPower power to apply to collector
     * @param indexerPower   power to apply to indexer
     * @param collector      subsystem
     * @param indexer        subsystem
     * @param pivot          subsystem (read only)
     * @param flywheel       subsystem
     * @param isAutonomous   whether or not this command is being used in autonomous
     */
    public SmartCollect(DoubleSupplier collectorPower, DoubleSupplier indexerPower, Collector collector,
            Indexer indexer, Pivot pivot, Flywheel flywheel, boolean isAutonomous) {
        this.collector = collector;
        this.indexer = indexer;
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collectorPower = collectorPower;
        this.indexerPower = indexerPower;

        if (isAutonomous || DriverStation.isAutonomous()) {
            addRequirements(collector, indexer);
        } else {
            addRequirements(collector, indexer, flywheel);
        }
    }

    // Overloaded constructor for use in teleop
    public SmartCollect(DoubleSupplier collectorPower, DoubleSupplier indexerPower, Collector collector,
            Indexer indexer, Pivot pivot, Flywheel flywheel) {
        this(collectorPower, indexerPower, collector, indexer, pivot, flywheel, false);
    }

    @Override
    public void initialize() {
        reversedFromExit = false;

        System.out.println("COLLECT - Smart Collect INIT");
    }

    @Override
    public void execute() {
        allowIndex = pivot.getAngle() < pivot.getMaxIndexAngle();

        switch (indexer.getPieceState()) {
            case NONE: /* Note not passing any beambreaks */
                reversedFromExit = false;
                collector.setPower(collectorPower.getAsDouble());
                if (allowIndex) {
                    indexer.setPower(indexerPower.getAsDouble());
                    if (!DriverStation.isAutonomous()) {
                        flywheel.setAllMotorsRPM(-200);
                    }
                }
                break;

            case IN_COLLECT: /* Note has passed beambreak past collector */
                if (allowIndex) {
                    // Slow down collector to prevent jamming
                    collector.setPower(collectorPower.getAsDouble());
                    indexer.setPower(indexerPower.getAsDouble());
                    if(!DriverStation.isAutonomous()){
                        flywheel.setAllMotorsRPM(-500);
                    }
                } else {
                    // Stop collecting since pivot is not in right place
                    collector.stop();
                    indexer.stop();
                }
                break;

            case IN_PIVOT: /* Note has touched entry indexer beambreak */
                collector.stop();
                if (allowIndex && !reversedFromExit) {
                    indexer.setPower(0.9 * indexerPower.getAsDouble());
                } else if (reversedFromExit) {
                    indexer.stop();
                    if (!DriverStation.isAutonomous()) {
                        flywheel.coast(true);
                    }
                    new TimedCommand(RobotContainer.hapticDriverCommand(), 1d).schedule();
                    new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
                }
                break;

            case IN_INDEXER: /* Note has touched exit indexer beambreak */
                collector.stop();
                indexer.setPower(-0.45 * indexerPower.getAsDouble());
                reversedFromExit = true;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
        indexer.stop();
        if(!DriverStation.isAutonomous()){
            flywheel.coast(true);
        }

        System.out.println("COLLECT - Smart Collect END");
    }

    @Override
    public boolean isFinished() {
        return reversedFromExit && indexer.getPieceState() == PieceState.IN_PIVOT;
    }
}
