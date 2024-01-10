package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

import frc.robot.command.Shoot;
import frc.robot.subsystem.Shooter;
import frc.thunder.LightningContainer;


public class RobotContainer extends LightningContainer {
    XboxController coPilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);

    Shooter shooter =  new Shooter();
    @Override
    protected void configureButtonBindings() {
        new Trigger(coPilot::getAButton).whileTrue(new Shoot(shooter));
    }

    @Override
    protected void configureDefaultCommands() {}

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
