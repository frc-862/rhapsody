package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMBER_STATES;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Climber extends SubsystemBase {

    private ThunderBird climbMotorR;
    private ThunderBird climbMotorL;

    private PositionVoltage setPointControl = new PositionVoltage(0d);
    private DutyCycleOut manualControl = new DutyCycleOut(0d);

    private CLIMBER_STATES state = CLIMBER_STATES.STOW;
    private Swerve drivetrain;

    private boolean hasClimbed = false;
    private boolean hasStowed = false;
    private boolean hasGroundedR = false;
    private boolean hasGroundedL = false;

    public Climber(Swerve drivetrain) {

        this.drivetrain = drivetrain;
        // configure climb motors
        climbMotorR = new ThunderBird(CAN.CLIMB_RIGHT, CAN.CANBUS_FD,
                ClimbConstants.CLIMB_RIGHT_MOTOR_INVERT, ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, ClimbConstants.CLIMB_MOTOR_BRAKE_MODE);
        climbMotorL = new ThunderBird(CAN.CLIMB_LEFT, CAN.CANBUS_FD,
            ClimbConstants.CLIMB_LEFT_MOTOR_INVERT, ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, ClimbConstants.CLIMB_MOTOR_BRAKE_MODE);

        climbMotorL.configPIDF(0, ClimbConstants.UNLOADED_KP, ClimbConstants.UNLOADED_KI, ClimbConstants.UNLOADED_KD);
        climbMotorL.configPIDF(1, ClimbConstants.LOADED_KP, ClimbConstants.LOADED_KI, ClimbConstants.LOADED_KD);

        climbMotorR.configPIDF(0, ClimbConstants.UNLOADED_KP, ClimbConstants.UNLOADED_KI, ClimbConstants.UNLOADED_KD);
        climbMotorR.configPIDF(1, ClimbConstants.LOADED_KP, ClimbConstants.LOADED_KI, ClimbConstants.LOADED_KD);

        FeedbackConfigs conf = new FeedbackConfigs();

        conf.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        conf.SensorToMechanismRatio = ClimbConstants.GEAR_REDUCTION;

        climbMotorL.applyConfig(climbMotorL.getConfig().withFeedback(conf));
        climbMotorR.applyConfig(climbMotorR.getConfig().withFeedback(conf));

        initLogging();

        climbMotorL.setPosition(0d);
        climbMotorR.setPosition(0d);
    }

    private void initLogging() { // TODO test and fix once we have climber
        LightningShuffleboard.setDoubleSupplier("Climb", "Left Height", () -> getHeightL());
        LightningShuffleboard.setDoubleSupplier("Climb", "Right Height", () -> getHeightR());
        LightningShuffleboard.setDoubleSupplier("Climb", "Left Setpoint", () -> getSetpointL());
        LightningShuffleboard.setDoubleSupplier("Climb", "Right Setpoint", () -> getSetpointR());
        LightningShuffleboard.setDoubleSupplier("Climb", "Left Applied", () -> climbMotorL.getMotorVoltage().getValueAsDouble());
        LightningShuffleboard.setDoubleSupplier("Climb", "Right Applied", () -> climbMotorR.getMotorVoltage().getValueAsDouble());
    }

    /**
     * Sets power to both climb motors
     * @param power the power to set both climb motors to
     */
    public void setPower(double power) {
        setPower(power, power);
    }

    /**
     * Sets power to climb motors
     * @param powerR the power to set the right climb motor to
     * @param powerL the power to set the left climb motor to
     */
    public void setPower(double powerL, double powerR) {
        climbMotorR.setControl(manualControl.withOutput(powerR)); // FOC On by default
        climbMotorL.setControl(manualControl.withOutput(powerL));
    }

    /**
     * Set power to left climb motor
     * @param power power
     */
    public void setPowerL(double power) {
        climbMotorL.set(power);
    }

    /**
     * Set power to right climb motor
     * @param power power
     */
    public void setPowerR(double power) {
        climbMotorL.set(power);
    }

    /**
     * sets the setpoint of both climb motors
     * @param setPoint setpoint for both climb motors in pulley rotations
     */
    public void setSetpoint(double setPoint) {
        setSetpoint(setPoint, setPoint);
    }

    /**
     * sets the setpoint of the climb motors
     * @param leftSetPoint setpoint for left climb motor in pulley rotations
     * @param rightSetPoint setpoint for right climb motor in pulley rotations
     */
    public void setSetpoint(double leftSetPoint, double rightSetPoint) {
        climbMotorL.setControl(setPointControl.withPosition(leftSetPoint));
        climbMotorR.setControl(setPointControl.withPosition(rightSetPoint));
    }

    /**
     * Stops all climb motors
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * @return height of right climb arm
     */
    public double getHeightR() {
        return climbMotorR.getPosition().getValueAsDouble();
    }

    /**
     * @return height of left climb arm
     */
    public double getHeightL() {
        return climbMotorL.getPosition().getValueAsDouble();
    }

    /**
     * @return the setpoint of the right climb arm
     */
    public double getSetpointR() {
        return this.setPointControl.Position;
    }

    /**
     * @return the setpoint of the left climb arm
     */
    public double getSetpointL() {
        return this.setPointControl.Position;
    }

    /**
     * stores state value for climber
     * @param state
     */
    public void setState(CLIMBER_STATES state) {
        this.state = state;
    }

    /**
     * @return current stored state of climber
     */
    public CLIMBER_STATES getState() {
        return state;
    }

    /**
     * tell climber if climber has begun climbing
     * @param hasClimbed boolean if the robot has climbed
     */
    public void setHasClimbed(boolean hasClimbed) {
        this.hasClimbed = hasClimbed;
    }

    /**
     * tell climber if the right arm has been fully extended (robot should have returned to ground
     * after climb)
     * @param hasGroundedR
     */
    public void setHasGroundedR(boolean hasGroundedR) {
        this.hasGroundedR = hasGroundedR;
    }

    /**
     * tell climber if the left arm has been fully extended (robot should have returned to ground
     * after climb)
     * @param hasGroundedL
     */
    public void setHasGroundedL(boolean hasGroundedL) {
        this.hasGroundedL = hasGroundedL;
    }

    /**
     * tell climber if both arms have been retracted after returning to gound
     * @param hasStowed
     */
    public void setHasStowed(boolean hasStowed) {
        this.hasStowed = hasStowed;
    }

    /**
     * resets has stowed/grounded/tipped values
     */
    public void resetHasValues() {
        hasGroundedL = false;
        hasGroundedR = false;
        hasStowed = false;
        hasClimbed = false;
    }

    @Override
    public void periodic() {
        // updates height based on limit switches
        for (TalonFX motor : new TalonFX[] {climbMotorR, climbMotorL}) {
            if (motor.getPosition().getValueAsDouble() > ClimbConstants.MAX_HEIGHT) {
                motor.setPosition(ClimbConstants.MAX_HEIGHT);
            }
            if (motor.getPosition().getValueAsDouble() < 0
                    || motor.getReverseLimit().getValueAsDouble() == 0) {
                motor.setPosition(0d);
            }
        }

        // updates states
        if (hasClimbed && getHeightL() < ClimbConstants.MAX_HEIGHT / 2
                && getHeightR() < ClimbConstants.MAX_HEIGHT / 2) {
            state = CLIMBER_STATES.CLIMBED;
            resetHasValues();
        }

        // ONCE the robot is completely climbed, the robot will be in the STOW state
        if (getHeightR() <= ClimbConstants.CLIMB_RETRACTION_TOLERANCE
                && getHeightL() <= ClimbConstants.CLIMB_RETRACTION_TOLERANCE
                && !drivetrain.isTipped() && hasStowed) {
            state = CLIMBER_STATES.STOW;
            resetHasValues();
        }

        if (ClimbConstants.MAX_HEIGHT - getHeightR() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE
                && ClimbConstants.MAX_HEIGHT
                        - getHeightL() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE
                && !drivetrain.isTipped() && hasGroundedR && hasGroundedL) {
            state = CLIMBER_STATES.GROUNDED;
            resetHasValues();
        }
    }
}
