package frc.robot.command.tests.testcmds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TurnTest extends Command {

    private Swerve drivetrain;
    public DoubleSupplier speed;

    /**
     * System test command for testing azimuth motors
     * @param drivetrain swerve subsystem
     * @param speed rotational rate
     */
    public TurnTest(Swerve drivetrain, DoubleSupplier speed) {
        this.drivetrain = drivetrain;
        this.speed = speed;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivetrain.setRobot(0d, 0d, speed.getAsDouble());//speed);
        System.out.println("ExecutingTurntest");
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.brake();
        System.out.println("Ending");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
