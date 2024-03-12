package frc.robot.command.tests;

import frc.robot.subsystems.Pivot;
import frc.thunder.testing.SystemTestCommand;

public class PivotAngleTest extends SystemTestCommand {

    private Pivot pivot;
    private double angle;

    /**
     * Sets pivot to a certain angle
     * 
     * @param pivot subsystem
     * @param angle angle to set pivot to (degrees)
     */
    public PivotAngleTest(Pivot pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void initializeTest() {
        pivot.setTargetAngle(angle);
    }

    @Override
    public void executeTest() {
    }

    @Override
    public void endTest(boolean interrupted) {
        pivot.setTargetAngle(pivot.getStowAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
