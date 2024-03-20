package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;

public class FlywheelIN extends Command {

    private final Flywheel flywheel;

    /**
     * Creates a new FlywheelIN.
     * @param flywheel subsystem
     */
    public FlywheelIN(Flywheel flywheel) {
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
    }

    @Override
    public void execute() {
        flywheel.setAllMotorsRPM(CandConstants.SOURCE_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.coast(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
