package frc.robot.command;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class SmartClimb extends Command {

    private Climber climber;
    private Swerve drivetrain;
    private PathFindToAuton pathFind;
    private LEDs leds;

    private DoubleSupplier leftPower;
    private DoubleSupplier rightPower;
    private BooleanSupplier bButton;

    private boolean buttonState;
    private boolean autoClimbEngaged;
    private boolean readyToLineUp;

    /**
     * SmartClimb to control the climber using the B button and sticks
     * @param climber subsystem
     * @param drivetrain subsystem
     * @param leftPower power to apply to left climber motor
     * @param rightPower power to apply to right climber motor
     * @param bButton B button state
     * @param leds subsystem
     */
    public SmartClimb(Climber climber, Swerve drivetrain, DoubleSupplier leftPower, DoubleSupplier rightPower, BooleanSupplier bButton, LEDs leds) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.bButton = bButton;
        this.leds = leds;
        this.buttonState = false;
        this.autoClimbEngaged = false;

        addRequirements(climber); // don't addrequirements for drivetrain because it's read only
    }

    @Override
    public void execute() {
        if (leftPower.getAsDouble() != 0d || rightPower.getAsDouble() != 0d) {
            // Engage Manual Climb whenever sticks are active
            climber.setPower(leftPower.getAsDouble(), rightPower.getAsDouble());
            autoClimbEngaged = false;
            readyToLineUp = true;
        } else if ((buttonState && !bButton.getAsBoolean()) || (drivetrain.isTipped()  && !autoClimbEngaged)) {
            // Auto retract on the falling edge of the B button or if the robot is tipped
            climber.retract();
            buttonState = false; // reset button state for next time
            if (drivetrain.isTipped()) {
                autoClimbEngaged = true; // trigger autoClimbEngaged to prevent auto deploy from re-engaging
            }
        } else if (bButton.getAsBoolean() && !autoClimbEngaged) {
            // line up if not currently lining up
            if (readyToLineUp) {
                lineUp();
            }
            // Auto deploy climb when B button is pressed and auto climb has not been engaged
            climber.deploy();
            buttonState = true;
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

    /**
     * Line up the robot to climb
     */
    public void lineUp() {
        readyToLineUp = false;

        // Determine which path to take based on the robot's current position
        double diffCenterX = Math.abs(drivetrain.getPose().getTranslation().getX() 
        - ClimbConstants.PATHFIND_CENTER_STAGE_START_POSE.getTranslation().getX());
        double diffCenterY = Math.abs(drivetrain.getPose().getTranslation().getY()
        - ClimbConstants.PATHFIND_CENTER_STAGE_START_POSE.getTranslation().getY());
        double diffCenter = Math.hypot(diffCenterX, diffCenterY);

        double diffHighX = Math.abs(drivetrain.getPose().getTranslation().getX()
        - ClimbConstants.PATHFIND_HIGH_STAGE_START_POSE.getTranslation().getX());
        double diffHighY = Math.abs(drivetrain.getPose().getTranslation().getY()
        - ClimbConstants.PATHFIND_HIGH_STAGE_START_POSE.getTranslation().getY());
        double diffHigh = Math.hypot(diffHighX, diffHighY);

        double diffLowX = Math.abs(drivetrain.getPose().getTranslation().getX()
        - ClimbConstants.PATHFIND_LOW_STAGE_START_POSE.getTranslation().getX());
        double diffLowY = Math.abs(drivetrain.getPose().getTranslation().getY()
        - ClimbConstants.PATHFIND_LOW_STAGE_START_POSE.getTranslation().getY());
        double diffLow = Math.hypot(diffLowX, diffLowY);

        // Create the pathfind command based on the shortest distance
        if (diffCenter < diffHigh && diffCenter < diffLow) {
            pathFind = new PathFindToAuton(
                PathPlannerPath.fromPathFile("Pathfind-CENTER-STAGE"), drivetrain);
        } else if (diffHigh < diffCenter && diffHigh < diffLow) {
            pathFind = new PathFindToAuton(
                PathPlannerPath.fromPathFile("Pathfind-HIGH-STAGE"), drivetrain);
        } else {
            pathFind = new PathFindToAuton(
                PathPlannerPath.fromPathFile("Pathfind-LOW-STAGE"), drivetrain);
        }
        
        pathFind.schedule();
    }
}
