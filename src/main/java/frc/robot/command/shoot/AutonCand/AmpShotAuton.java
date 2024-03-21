package frc.robot.command.shoot.AutonCand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Indexer;

public class AmpShotAuton extends Command {

    private final Flywheel flywheel;
    private final Pivot pivot;
    private final Indexer indexer;

    private boolean shot = false;
    private double startTime = 0;
    private double shotTime = 0;

    private boolean startIndexing = false;

    /**
     * Creates a new AmpShot
     *
     * @param flywheel subsystem
     * @param pivot    subsystem
     * @param indexer  subsystem
     */
    public AmpShotAuton(Flywheel flywheel, Pivot pivot, Indexer indexer) {
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.indexer = indexer;

        addRequirements(flywheel, pivot, indexer);
    }

    @Override
    public void initialize() {
        shot = false;
        startIndexing = false;
        startTime = Timer.getFPGATimestamp();
        flywheel.setTopMotorRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
        flywheel.setBottomMotorRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
        pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
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
            shot = true;
            shotTime = Timer.getFPGATimestamp();
            indexer.indexUp();
        }

        flywheel.setTopMotorRPM(CandConstants.AMP_TOP_RPM + flywheel.getBias());
        flywheel.setBottomMotorRPM(CandConstants.AMP_BOTTOM_RPM + flywheel.getBias());
        pivot.setTargetAngle(CandConstants.AMP_ANGLE + pivot.getBias());
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.coast(true);
        pivot.setTargetAngle(pivot.getStowAngle());
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return shot && shotTime - startTime >= CandConstants.TIME_TO_SHOOT;
    }
}
