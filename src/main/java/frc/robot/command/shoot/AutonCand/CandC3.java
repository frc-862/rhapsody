package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Indexer;

public class CandC3 extends Command {

    private final Flywheel flywheel;
    private final Pivot pivot;
    private final Indexer indexer;

    private boolean startIndexing = false;

    /**
     * Creates a new CandC2.
     *
     * @param flywheel subsystem
     * @param pivot    subsystem
     * @param indexer  subsystem
     */
    public CandC3(Flywheel flywheel, Pivot pivot, Indexer indexer) {
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.indexer = indexer;

        addRequirements(flywheel, pivot, indexer);
    }

    @Override
    public void initialize() {
        flywheel.setAllMotorsRPM(CandConstants.C3_RPM);
        pivot.setTargetAngle(CandConstants.C3_ANGLE);
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
        }

        flywheel.setAllMotorsRPM(CandConstants.C3_RPM);
        pivot.setTargetAngle(CandConstants.C3_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.coast(false);
        pivot.setTargetAngle(pivot.getStowAngle());
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return !indexer.hasNote();
    }
}
