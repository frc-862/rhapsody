package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.robot.subsystems.Collector;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collect extends Command {

    // Declares collector
    private DoubleSupplier powerSupplier;
    private Collector collector;

    /**
     * Creates a new Collect.
     *
     * @param powerSupplier DoubleSupplier for power of motor (-1 to 1)
     * @param collector     subsystem
     */
    public Collect(DoubleSupplier powerSupplier, Collector collector) {
        this.collector = collector;
        this.powerSupplier = powerSupplier;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void execute() {
        collector.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
