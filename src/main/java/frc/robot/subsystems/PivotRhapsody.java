package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.RhapsodyPivotConstants;

public class PivotRhapsody extends SubsystemBase implements Pivot {

    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    private double bias = 0;
    private double targetAngle = RhapsodyPivotConstants.STOW_ANGLE;

    private DoubleLogEntry currentAngleLog;
    private DoubleLogEntry targetAngleRotLog;
    private DoubleLogEntry targetAngleDegLog;
    private BooleanLogEntry onTargetLog;
    private DoubleLogEntry biasLog;
    private BooleanLogEntry forwardLimitLog;
    private BooleanLogEntry reverseLimitLog;
    private DoubleLogEntry powerLog;

    public PivotRhapsody() {
        System.out.println("RHAPSODY PIVOT");
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(RhapsodyPivotConstants.ENCODER_OFFSET)
                .withSensorDirection(RhapsodyPivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        angleMotor = new ThunderBird(CAN.PIVOT_ANGLE_MOTOR, CAN.CANBUS_FD, RhapsodyPivotConstants.MOTOR_INVERT,
                RhapsodyPivotConstants.MOTOR_STATOR_CURRENT_LIMIT, RhapsodyPivotConstants.MOTOR_BRAKE_MODE);
        TalonFXConfiguration motorConfig = angleMotor.getConfig();
        motorConfig.Slot0.kP = RhapsodyPivotConstants.MOTOR_KP;
        motorConfig.Slot0.kI = RhapsodyPivotConstants.MOTOR_KI;
        motorConfig.Slot0.kD = RhapsodyPivotConstants.MOTOR_KD;
        motorConfig.Slot0.kS = RhapsodyPivotConstants.MOTOR_KS;
        motorConfig.Slot0.kV = RhapsodyPivotConstants.MOTOR_KV;
        motorConfig.Slot0.kA = RhapsodyPivotConstants.MOTOR_KA;
        motorConfig.Slot0.kG = RhapsodyPivotConstants.MOTOR_KG;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = RhapsodyPivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = RhapsodyPivotConstants.ROTOR_TO_ENCODER_RATIO;

        // MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity =
        // PivotConstants.MAGIC_CRUISE_VEL;
        // motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAGIC_ACCEL;
        // motionMagicConfigs.MotionMagicJerk = PivotConstants.MAGIC_JERK;

        angleMotor.applyConfig(motorConfig);

        setTargetAngle(targetAngle);

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        currentAngleLog = new DoubleLogEntry(log, "/Pivot/CurrentAngle");
        targetAngleRotLog = new DoubleLogEntry(log, "/Pivot/TargetAngleRot");
        targetAngleDegLog = new DoubleLogEntry(log, "/Pivot/TargetAngleDeg");

        onTargetLog = new BooleanLogEntry(log, "/Pivot/OnTarget");

        biasLog = new DoubleLogEntry(log, "/Pivot/Bias");

        forwardLimitLog = new BooleanLogEntry(log, "/Pivot/ForwardLimit");
        reverseLimitLog = new BooleanLogEntry(log, "/Pivot/ReverseLimit");

        powerLog = new DoubleLogEntry(log, "/Pivot/Power");

        LightningShuffleboard.setDoubleSupplier("Pivot", "CurrentAngle", () -> getAngle() * 360);
        LightningShuffleboard.setDoubleSupplier("Pivot", "TargetAngle", () -> targetAngle * 360);
        LightningShuffleboard.setBoolSupplier("Pivot", "OnTarget", () -> onTarget());

        LightningShuffleboard.setDoubleSupplier("Pivot", "Bias", () -> bias);
    }

    @Override
    public void periodic() {

        // angleMotor.getConfig().Slot0.kP = LightningShuffleboard.getDouble("Pivot",
        // "kP", PivotConstants.MOTOR_KP);
        // angleMotor.getConfig().Slot0.kI = LightningShuffleboard.getDouble("Pivot",
        // "kI", PivotConstants.MOTOR_KI);
        // angleMotor.getConfig().Slot0.kD = LightningShuffleboard.getDouble("Pivot",
        // "kD", PivotConstants.MOTOR_KD);
        // angleMotor.getConfig().Slot0.kS = LightningShuffleboard.getDouble("Pivot",
        // "kS", PivotConstants.MOTOR_KS);
        // angleMotor.getConfig().Slot0.kV = LightningShuffleboard.getDouble("Pivot",
        // "kV", PivotConstants.MOTOR_KV);
        // angleMotor.getConfig().Slot0.kA = LightningShuffleboard.getDouble("Pivot",
        // "kA", PivotConstants.MOTOR_KA);

        angleMotor.getConfig().MotionMagic.MotionMagicCruiseVelocity = LightningShuffleboard.getDouble("Pivot",
                "cruiseVelocity", RhapsodyPivotConstants.MAGIC_CRUISE_VEL);
        angleMotor.getConfig().MotionMagic.MotionMagicAcceleration = LightningShuffleboard.getDouble("Pivot",
                "acceleration", RhapsodyPivotConstants.MAGIC_ACCEL);
        angleMotor.getConfig().MotionMagic.MotionMagicJerk = LightningShuffleboard.getDouble("Pivot", "jerk",
                RhapsodyPivotConstants.MAGIC_JERK);

        // pivotTuner.update();

        // setTargetAngle(LightningShuffleboard.getDouble("Pivot", "settargetAngle",
        // targetAngle));

        // SETS angle to angle of limit switch on press
        if (getForwardLimit()) {
            resetAngle(RhapsodyPivotConstants.MIN_ANGLE);
        } else if (getReverseLimit()) {
            resetAngle(RhapsodyPivotConstants.MAX_ANGLE);
        }

        moveToTarget();

        updateLogging();
    }

    /**
     * update logging
     */
    public void updateLogging() {
        currentAngleLog.append(getAngle());
        targetAngleRotLog.append(targetAngle);
        targetAngleDegLog.append(targetAngle * 360);

        onTargetLog.append(onTarget());

        biasLog.append(bias);

        forwardLimitLog.append(getForwardLimit());
        reverseLimitLog.append(getReverseLimit());

        powerLog.append(angleMotor.get());
    }

    /**
     * Sets the target angle of the pivot
     * @param angle Angle of the pivot in degrees
     */
    public void setTargetAngle(double angle) {
        targetAngle = (MathUtil.clamp(angle + bias, RhapsodyPivotConstants.MIN_ANGLE, RhapsodyPivotConstants.MAX_ANGLE) / 360);
    }

    /*
     * Moves pivot motor toward target angle
     */
    private void moveToTarget() {
        angleMotor.setControl(anglePID.withPosition(targetAngle));
    }

    public void setPower(double power) {
        angleMotor.set(power);
    }

    /**
     * @return The current angle of the pivot in rotations
     */
    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    /**
     * @return Whether or not the pivot is on target, within Angle tolerance
     */
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < RhapsodyPivotConstants.ANGLE_TOLERANCE;
    }

    /**
     * Gets forward limit switch
     * @return true if pressed
     */
    public boolean getForwardLimit() {
        return false; //angleMotor.getForwardLimit().refresh().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets reverse limit switch
     * @return true if pressed
     */
    public boolean getReverseLimit() {
        return false; //angleMotor.getReverseLimit().refresh().getValue() == ReverseLimitValue.ClosedToGround;
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
        bias += RhapsodyPivotConstants.BIAS_INCREMENT;
    }

    /**
     * Decreases the bias of the pivot by set amount
     */
    public void decreaseBias() {
        bias -= RhapsodyPivotConstants.BIAS_INCREMENT;
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
    public double getStowAngle() {
        return RhapsodyPivotConstants.STOW_ANGLE;
    }

    /**
     * @return max Index Angle
     */
    public double getMaxIndexAngle() {
        return RhapsodyPivotConstants.MAX_INDEX_ANGLE;
    }
}
