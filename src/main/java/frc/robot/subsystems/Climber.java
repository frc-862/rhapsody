package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMBER_STATES;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Pair;

public class Climber extends SubsystemBase {

    private ThunderBird climbMotorR;
    private ThunderBird climbMotorL;

    private PositionVoltage setPointControlR = new PositionVoltage(0d);
    private PositionVoltage setPointControlL = new PositionVoltage(0d);

    private DutyCycleOut manualControl = new DutyCycleOut(0d);

    private DoubleLogEntry leftHeightLog;
    private DoubleLogEntry rightHeightLog;
    private DoubleLogEntry leftSetpointLog;
    private DoubleLogEntry rightSetpointLog;
    private DoubleLogEntry leftAppliedLog;
    private DoubleLogEntry rightAppliedLog;

    public Climber() {
        // configure climb motors
        climbMotorR = new ThunderBird(CAN.CLIMB_RIGHT, CAN.CANBUS_FD,
                ClimbConstants.CLIMB_RIGHT_MOTOR_INVERT, ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, ClimbConstants.CLIMB_MOTOR_BRAKE_MODE);
        climbMotorL = new ThunderBird(CAN.CLIMB_LEFT, CAN.CANBUS_FD,
            ClimbConstants.CLIMB_LEFT_MOTOR_INVERT, ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, ClimbConstants.CLIMB_MOTOR_BRAKE_MODE);

        climbMotorL.configPIDF(0, ClimbConstants.UNLOADED_KP, ClimbConstants.UNLOADED_KI, ClimbConstants.UNLOADED_KD);
        climbMotorL.configPIDF(1, ClimbConstants.LOADED_KP, ClimbConstants.LOADED_KI, ClimbConstants.LOADED_KD);

        climbMotorR.configPIDF(0, ClimbConstants.UNLOADED_KP, ClimbConstants.UNLOADED_KI, ClimbConstants.UNLOADED_KD);
        climbMotorR.configPIDF(1, ClimbConstants.LOADED_KP, ClimbConstants.LOADED_KI, ClimbConstants.LOADED_KD);

        FeedbackConfigs sensorConf = new FeedbackConfigs();
        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs();

        sensorConf.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        sensorConf.SensorToMechanismRatio = ClimbConstants.GEAR_REDUCTION;

        softLimitConf.ForwardSoftLimitEnable = true;
        softLimitConf.ForwardSoftLimitThreshold = ClimbConstants.MAX_HEIGHT;

        climbMotorL.applyConfig(climbMotorL.getConfig().withFeedback(sensorConf).withSoftwareLimitSwitch(softLimitConf));
        climbMotorR.applyConfig(climbMotorR.getConfig().withFeedback(sensorConf).withSoftwareLimitSwitch(softLimitConf));

        climbMotorL.setPosition(0d);
        climbMotorR.setPosition(0d);

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() { // TODO test and fix once we have climber
        DataLog log = DataLogManager.getLog();

        leftHeightLog = new DoubleLogEntry(log, "/Climb/LeftHeight");
        rightHeightLog = new DoubleLogEntry(log, "/Climb/RightHeight");
        leftSetpointLog = new DoubleLogEntry(log, "/Climb/LeftSetpoint");
        rightSetpointLog = new DoubleLogEntry(log, "/Climb/RightSetpoint");
        leftAppliedLog = new DoubleLogEntry(log, "/Climb/LeftApplied");
        rightAppliedLog = new DoubleLogEntry(log, "/Climb/RightApplied");
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
        setPointControlL = setPointControlL.withPosition(leftSetPoint);
        setPointControlR = setPointControlR.withPosition(rightSetPoint);
        climbMotorL.setControl(setPointControlL);
        climbMotorR.setControl(setPointControlR);
    }

    /**
     * sets the setpoint of the climb motors to the max height
     */
    public void deploy() {
        setSetpoint(ClimbConstants.MAX_HEIGHT);
    }

    /**
     * sets the setpoint of the climb motors to the retracted position
     */
    public void retract() {
        setSetpoint(ClimbConstants.CLIMB_PID_SETPOINT_RETRACTED);
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
        return this.setPointControlR.Position;
    }

    /**
     * @return the setpoint of the left climb arm
     */
    public double getSetpointL() {
        return this.setPointControlL.Position;
    }

    @Override
    public void periodic() {
        // zeroes height if the limit switch is pressed or position is negative
        for (TalonFX motor : new TalonFX[] {climbMotorR, climbMotorL}) {
            if (motor.getPosition().getValueAsDouble() < 0
                    || motor.getReverseLimit().getValueAsDouble() == 0) {
                motor.setPosition(0d);
            }
        }

        updateLogging();
    }

    /**
     * update logging
     */
    public void updateLogging() {
        leftHeightLog.append(getHeightL());
        rightHeightLog.append(getHeightR());
        leftSetpointLog.append(getSetpointL());
        rightSetpointLog.append(getSetpointR());
        leftAppliedLog.append(climbMotorL.getMotorVoltage().getValueAsDouble());
        rightAppliedLog.append(climbMotorR.getMotorVoltage().getValueAsDouble());
    }
}
