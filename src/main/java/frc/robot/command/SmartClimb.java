package frc.robot.command;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;

public class SmartClimb extends Command {
    private Climber climber;
    private Swerve drivetrain;
    private DoubleSupplier leftPower;
    private DoubleSupplier rightPower;
    private BooleanSupplier bButton;
    private boolean buttonState;
    private boolean autoClimbEngaged;


    public SmartClimb(Climber climber, Swerve drivetrain, DoubleSupplier leftPower, DoubleSupplier rightPower, BooleanSupplier bButton) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.bButton = bButton;
        this.buttonState = false;
        this.autoClimbEngaged = false;
        addRequirements(climber); //don't addrequirements for drivetrain because it's read only
    }

    @Override
    public void execute() {
        if (leftPower.getAsDouble() != 0d || rightPower.getAsDouble() != 0d) {
            //Engage Manual Climb whenever sticks are active
            climber.setPower(leftPower.getAsDouble(), rightPower.getAsDouble());
            autoClimbEngaged = false;
            System.out.println("CLIMB MANUAL");
        } else if (bButton.getAsBoolean() && !autoClimbEngaged) {
            //Auto deploy climb when B button is pressed and auto climb has not been engaged
            climber.deploy();
            buttonState = true;
            System.out.println("CLIMB DEPLOY");
        } else if ((buttonState && !bButton.getAsBoolean()) || drivetrain.isTipped()) {
            //Auto retract on the falling edge of the B button or if the robot is tipped
            climber.retract();
            buttonState = false;
            if (drivetrain.isTipped()) {
                autoClimbEngaged = true;
            }
            System.out.println("CLIMB RETRACT");
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.isTipped();
    }
}
