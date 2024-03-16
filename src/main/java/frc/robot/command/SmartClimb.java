package frc.robot.command;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class SmartClimb extends Command {

    private Climber climber;
    private Swerve drivetrain;
    private LEDs leds;

    private DoubleSupplier leftPower;
    private DoubleSupplier rightPower;
    private BooleanSupplier bButton;

    private boolean buttonState;
    private boolean autoClimbEngaged;

    private Pivot pivot;

    /**
     * SmartClimb to control the climber using the B button and sticks
     * @param climber subsystem, requires
     * @param drivetrain subsystem
     * @param pivot subsystem, requires
     * @param leds subsystem
     * @param leftPower power to apply to left climber motor
     * @param rightPower power to apply to right climber motor
     * @param bButton B button state
     
     */
    public SmartClimb(Climber climber, Swerve drivetrain, Pivot pivot, LEDs leds, DoubleSupplier leftPower, DoubleSupplier rightPower, BooleanSupplier bButton) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.leds = leds;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.bButton = bButton;
        this.buttonState = false;

        addRequirements(climber, pivot); // don't addrequirements for drivetrain because it's read only
    }

    @Override
    public void initialize() {
        autoClimbEngaged = false;
        pivot.setTargetAngle(pivot.getStowAngle());
    }

    @Override
    public void execute() {
        if (pivot.onTarget()) {
            autoClimbEngaged = false;
        } else {
            if (leftPower.getAsDouble() != 0d || rightPower.getAsDouble() != 0d) {
                // Engage Manual Climb whenever sticks are active
                climber.setPower(leftPower.getAsDouble(), rightPower.getAsDouble());
                autoClimbEngaged = false;
            } else if ((buttonState && !bButton.getAsBoolean()) || (drivetrain.isTipped() && !autoClimbEngaged)) {
                // Auto retract on the falling edge of the B button or if the robot is tipped
                climber.retract();
                buttonState = false; // reset button state for next time
                if (drivetrain.isTipped()) {
                    autoClimbEngaged = true; // trigger autoClimbEngaged to prevent auto deploy from re-engaging
                }
            } else if (bButton.getAsBoolean() && !autoClimbEngaged) {
                // Auto deploy climb when B button is pressed and auto climb has not been
                // engaged
                climber.deploy();
                buttonState = true;
            }
        } else if (bButton.getAsBoolean() && !autoClimbEngaged) {
            // Auto deploy climb when B button is pressed and auto climb has not been engaged
            climber.deploy();
            buttonState = true;
        } else if (climber.isManual()) {
            climber.setPower(0d);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        leds.enableState(LED_STATES.FINISHED_CLIMB);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
