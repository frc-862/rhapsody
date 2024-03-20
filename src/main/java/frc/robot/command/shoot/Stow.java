package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Flywheel;

public class Stow extends Command {

    private Pivot pivot;
    private Flywheel flywheel;

    /**
     * Creates a new Stow.
     *
     * @param pivot    subsystem
     * @param flywheel subsystem
     */
    public Stow(Flywheel flywheel, Pivot pivot) {
        this.pivot = pivot;
        this.flywheel = flywheel;

        addRequirements(pivot, flywheel);
    }

    @Override
    public void initialize() {
        pivot.setTargetAngle(pivot.getStowAngle());
        flywheel.coast(true);
    }

    @Override
    public void execute() {
        pivot.setTargetAngle(pivot.getStowAngle());
    }

    @Override
    public boolean isFinished() {
        return pivot.onTarget();
    }
}