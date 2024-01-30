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
     * @param leftPower supplier for left motor power
     * @param rightPower supplier for right motor power
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
