package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;
import frc.robot.Constants.MercuryPivotConstants;

public class PivotMercury extends SubsystemBase implements Pivot {

    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    // private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    // private final MotionMagicVoltage motionMagicPID = new MotionMagicVoltage(0);
    private final PIDController angleController = new PIDController(0.06, 0, 0);
    private double bias = 0;

    private double targetAngle = MercuryPivotConstants.STOW_ANGLE;

    public PivotMercury() {
        System.out.println("MERCURY PIVOT");
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(MercuryPivotConstants.ENCODER_OFFSET)
                .withSensorDirection(MercuryPivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        angleMotor = new ThunderBird(CAN.PIVOT_ANGLE_MOTOR, CAN.CANBUS_FD, MercuryPivotConstants.MOTOR_INVERT,
                MercuryPivotConstants.MOTOR_STATOR_CURRENT_LIMIT, MercuryPivotConstants.MOTOR_BRAKE_MODE);
        angleMotor.configPIDF(0, MercuryPivotConstants.MOTOR_KP, MercuryPivotConstants.MOTOR_KI,
                MercuryPivotConstants.MOTOR_KD, MercuryPivotConstants.MOTOR_KS, MercuryPivotConstants.MOTOR_KV);
        TalonFXConfiguration motorConfig = angleMotor.getConfig();

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = MercuryPivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = MercuryPivotConstants.ROTOR_TO_ENCODER_RATIO;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MercuryPivotConstants.MAGIC_CRUISE_VEL;
        motionMagicConfigs.MotionMagicAcceleration = MercuryPivotConstants.MAGIC_ACCEL;
        motionMagicConfigs.MotionMagicJerk = MercuryPivotConstants.MAGIC_JERK;

        angleMotor.applyConfig(motorConfig);

        angleController.setIntegratorRange(0.02, 1);
        angleController.setTolerance(MercuryPivotConstants.ANGLE_TOLERANCE);

        initLogging();
        setTargetAngle(targetAngle);
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Pivot", "Current Angle", () -> getAngle());
        LightningShuffleboard.setDoubleSupplier("Pivot", "Target Angle", () -> targetAngle);

        LightningShuffleboard.setBoolSupplier("Pivot", "On target", () -> onTarget());

        LightningShuffleboard.setDoubleSupplier("Pivot", "Bias", this::getBias);

        LightningShuffleboard.setBoolSupplier("Pivot", "Forward Limit", () -> getForwardLimit());
        LightningShuffleboard.setBoolSupplier("Pivot", "Reverse Limit", () -> getReverseLimit());

        LightningShuffleboard.setDoubleSupplier("Pivot", "Power", () -> angleMotor.get());

    }

    @Override
    public void periodic() {
        angleMotor.getConfig().MotionMagic.MotionMagicCruiseVelocity = LightningShuffleboard.getDouble("Pivot",
                "cruiseVelocity", MercuryPivotConstants.MAGIC_CRUISE_VEL);
        angleMotor.getConfig().MotionMagic.MotionMagicAcceleration = LightningShuffleboard.getDouble("Pivot",
                "acceleration", MercuryPivotConstants.MAGIC_ACCEL);
        angleMotor.getConfig().MotionMagic.MotionMagicJerk = LightningShuffleboard.getDouble("Pivot", "jerk",
                MercuryPivotConstants.MAGIC_JERK);

        // pivotTuner.update();

        // setTargetAngle(LightningShuffleboard.getDouble("Pivot", "settargetAngle",
        // targetAngle));

        // SETS angle to angle of limit switch on press
        if (getForwardLimit()) {
            resetAngle(MercuryPivotConstants.MIN_ANGLE);
        } else if (getReverseLimit()) {
            resetAngle(MercuryPivotConstants.MAX_ANGLE);
        }

        moveToTarget();

    }

    /**
     * Sets the target angle of the pivot
     * 
     * @param angle Angle of the pivot
     */
    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle + bias, MercuryPivotConstants.MIN_ANGLE, MercuryPivotConstants.MAX_ANGLE);
    }

    /*
     * Moves pivot motor toward target angle
     */
    private void moveToTarget() {
        double pidOutput = angleController.calculate(getAngle(), targetAngle);
        setPower(pidOutput);
    }

    public void setPower(double power) {
        angleMotor.set(power);
    }

    /**
     * @return The current angle of the pivot in degrees
     */
    public double getAngle() {
        return (angleMotor.getPosition().getValue() * 360) % 360;
    }

    /**
     * @return Whether or not the pivot is on target, within Angle tolerance
     */
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < MercuryPivotConstants.ANGLE_TOLERANCE;
    }

    /**
     * Gets forward limit switch
     * 
     * @return true if pressed
     */
    public boolean getForwardLimit() {
        return angleMotor.getForwardLimit().refresh().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets reverse limit switch
     * 
     * @return true if pressed
     */
    public boolean getReverseLimit() {
        return angleMotor.getReverseLimit().refresh().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * @return The bias to add to the target angle of the pivot
     */
    public double getBias() {
        return bias;
    }

    /**
     * Increases the bias of the pivot by set amount
     */
    public void increaseBias() {
        bias += MercuryPivotConstants.BIAS_INCREMENT;
    }

    /**
     * Decreases the bias of the pivot by set amount
     */
    public void decreaseBias() {
        bias -= MercuryPivotConstants.BIAS_INCREMENT;
    }

    /**
     * Resets the bias of the pivot
     */
    public void resetBias() {
        bias = 0;
    }

    /**
     * @param angle angle to set the pivot angle to
     */
    public void resetAngle(double angle) {
        // TODO is this necessary and implement
    }

    /**
     * @return current stow angle
     */
    public double getStowAlnge() {
        return MercuryPivotConstants.STOW_ANGLE;
    }

    /**
     * @return max Index Angle
     */
    public double getMaxIndexAngle() {
        return MercuryPivotConstants.MAX_INDEX_ANGLE;
    }
}