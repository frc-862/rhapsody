package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Indexer;

public class Cand35 extends Command {

    private final Flywheel flywheel;
    private final Pivot pivot;
    private final Indexer indexer;
    private final Collector collector;

    private boolean startIndexing = false;

    /**
     * Creates a new CandLine.
     *
     * @param flywheel  subsystem
     * @param pivot     subsystem
     * @param indexer   subsystem
     * @param collector subsystem
     */
    public Cand35(Flywheel flywheel, Pivot pivot, Indexer indexer, Collector collector) {
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.indexer = indexer;
        this.collector = collector;

        addRequirements(flywheel, pivot, indexer, collector);
    }

    @Override
    public void initialize() {
        startIndexing = false;
        flywheel.setAllMotorsRPM(CandConstants.THREEFIVE_RPM);
        pivot.setTargetAngle(CandConstants.THREEFIVE_ANGLE);
    }

    @Override
    public void execute() {
        // Checks if the pivot and flywheel are on target then shoots
        // also checks whether or not the flywheel's target RPM is greater than 0
        if (pivot.onTarget() && flywheel.allMotorsOnTarget()
                && (flywheel.getTopMotorRPM() != 0 && flywheel.getBottomMotorRPM() != 0)) {
            startIndexing = true;
        }

        if (startIndexing) {
            indexer.indexUp();
            collector.setPower(1d);
        }

        flywheel.setAllMotorsRPM(CandConstants.THREEFIVE_RPM);
        pivot.setTargetAngle(CandConstants.THREEFIVE_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.coast(true);
        pivot.setTargetAngle(pivot.getStowAngle());
        indexer.stop();
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return flywheel.getKama();
    }
}
