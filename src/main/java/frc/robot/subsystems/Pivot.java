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
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    private double bias = 0;
    private double targetAngle = PivotConstants.STOW_ANGLE;

    public Pivot() {
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(PivotConstants.ENCODER_OFFSET)
                .withSensorDirection(PivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        angleMotor = new ThunderBird(CAN.PIVOT_ANGLE_MOTOR, CAN.CANBUS_FD, PivotConstants.MOTOR_INVERT,
                        PivotConstants.MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.MOTOR_BRAKE_MODE);
        TalonFXConfiguration motorConfig = angleMotor.getConfig();
        motorConfig.Slot0.kP = PivotConstants.MOTOR_KP;
        motorConfig.Slot0.kI = PivotConstants.MOTOR_KI;
        motorConfig.Slot0.kD = PivotConstants.MOTOR_KD;
        motorConfig.Slot0.kS = 0d;
        motorConfig.Slot0.kV = 0d;
        motorConfig.Slot0.kA = 0d;
        motorConfig.Slot0.kG = PivotConstants.MOTOR_KG;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.ROTOR_TO_ENCODER_RATIO;

        // MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.MAGIC_CRUISE_VEL;
        // motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAGIC_ACCEL;
        // motionMagicConfigs.MotionMagicJerk = PivotConstants.MAGIC_JERK;

        angleMotor.applyConfig(motorConfig);

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

        // angleMotor.getConfig().Slot0.kP = LightningShuffleboard.getDouble("Pivot", "kP", PivotConstants.MOTOR_KP);
        // angleMotor.getConfig().Slot0.kI = LightningShuffleboard.getDouble("Pivot", "kI", PivotConstants.MOTOR_KI);
        // angleMotor.getConfig().Slot0.kD = LightningShuffleboard.getDouble("Pivot", "kD", PivotConstants.MOTOR_KD);
        // angleMotor.getConfig().Slot0.kS = LightningShuffleboard.getDouble("Pivot", "kS", PivotConstants.MOTOR_KS);
        // angleMotor.getConfig().Slot0.kV = LightningShuffleboard.getDouble("Pivot", "kV", PivotConstants.MOTOR_KV);
        // angleMotor.getConfig().Slot0.kA = LightningShuffleboard.getDouble("Pivot", "kA", PivotConstants.MOTOR_KA);

        angleMotor.getConfig().MotionMagic.MotionMagicCruiseVelocity = LightningShuffleboard.getDouble("Pivot", "cruiseVelocity", PivotConstants.MAGIC_CRUISE_VEL);
        angleMotor.getConfig().MotionMagic.MotionMagicAcceleration = LightningShuffleboard.getDouble("Pivot", "acceleration", PivotConstants.MAGIC_ACCEL);
        angleMotor.getConfig().MotionMagic.MotionMagicJerk = LightningShuffleboard.getDouble("Pivot", "jerk", PivotConstants.MAGIC_JERK);

        // pivotTuner.update();

        // setTargetAngle(LightningShuffleboard.getDouble("Pivot", "settargetAngle", targetAngle));

        // SETS angle to angle of limit switch on press
        if (getForwardLimit()) {
            resetAngle(PivotConstants.MIN_ANGLE);
        } else if(getReverseLimit()) {
            resetAngle(PivotConstants.MAX_ANGLE);
        }

        moveToTarget();

    }

    /**
     * Sets the target angle of the pivot
     * @param angle Angle of the pivot
     */
    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle + bias, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE);
        angleMotor.setControl(anglePID.withPosition(targetAngle));
    }

    /*
     * Moves pivot motor toward target angle
     */
    private void moveToTarget(){
        angleMotor.setControl(anglePID.withPosition(targetAngle));
    }

    public void setPower(double power){
        angleMotor.set(power);
    }

    /**
     * @return The current angle of the pivot in degrees
     */
    public double getAngle() {
        return angleMotor.getPosition().getValue() * 360;
    }

    /**
     * @return Whether or not the pivot is on target, within Angle tolerance
     */
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < PivotConstants.ANGLE_TOLERANCE;
    }

    /**
     * Gets forward limit switch
     * @return true if pressed
     */
    public boolean getForwardLimit() {
        return angleMotor.getForwardLimit().refresh().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets reverse limit switch
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
        bias += PivotConstants.BIAS_INCREMENT;
    }

    /**
     * Decreases the bias of the pivot by set amount
     */
    public void decreaseBias() {
        bias -= PivotConstants.BIAS_INCREMENT;
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
}
