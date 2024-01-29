package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ManualClimb extends Command {

    private Climber climber;
    private DoubleSupplier leftPower;
    private DoubleSupplier rightPower;

    /**
     * Creates a new ManualClimb.
     * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
     * @param climber subsystem
     */
    public ManualClimb(DoubleSupplier leftPower, DoubleSupplier rightPower, Climber climber) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.climber = climber;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPower(leftPower.getAsDouble(), rightPower.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
