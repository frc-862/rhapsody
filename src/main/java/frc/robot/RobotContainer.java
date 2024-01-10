package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.*;
import frc.robot.subsystem.*;
import frc.thunder.LightningContainer;

public class RobotContainer extends LightningContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    XboxController driver = new XboxController(0); // My joystick

    // Create subsystems
    Collector collector = new Collector();

    @Override
    protected void configureButtonBindings() {
        new Trigger(driver::getRightBumper).whileTrue(new Collect(collector));
    }

    @Override
    protected void configureDefaultCommands() {
        // Get collector entry beam break state, then run collector if object is present
        collector.setDefaultCommand(new RunCommand(() -> {
            if (!collector.getEntryBeamBreakState()) {
                collector.setPower(1d);
            } else {
                collector.stop();
            }
        }, collector));
    }

    @Override
    protected void configureAutonomousCommands() {}

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected void configureSystemTests() {}
}
