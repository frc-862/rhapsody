package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;


public class SetPointClimb extends Command {
    private Climber climber;
    private DoubleSupplier setPointL;
    private DoubleSupplier setPointR;

    public SetPointClimb(Climber climber, DoubleSupplier setPointL, DoubleSupplier setPointR) {
        addRequirements(climber);
        this.climber = climber;
        this.setPointL = setPointL;
        this.setPointR = setPointR;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climber.setSetpoint(setPointL.getAsDouble(), setPointR.getAsDouble());
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
