package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    private double bias = 0;

    private double targetAngle = 0; // TODO: find initial target angle

    public Pivot() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(PivotConstants.ENCODER_OFFSET).withSensorDirection(PivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        motorConfig.MotorOutput.Inverted = PivotConstants.PIVOT_MOTOR_INVERT ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = PivotConstants.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT > 0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = PivotConstants.PIVOT_MOTOR_STATOR_CURRENT_LIMIT > 0;
        motorConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.PIVOT_MOTOR_STATOR_CURRENT_LIMIT;
        motorConfig.MotorOutput.NeutralMode = PivotConstants.PIVOT_MOTOR_NEUTRAL_MODE;

        motorConfig.Slot0.kP = PivotConstants.PIVOT_MOTOR_KP;
        motorConfig.Slot0.kI = PivotConstants.PIVOT_MOTOR_KI;
        motorConfig.Slot0.kD = PivotConstants.PIVOT_MOTOR_KD;
        motorConfig.Slot0.kS = PivotConstants.PIVOT_MOTOR_KS;
        motorConfig.Slot0.kV = PivotConstants.PIVOT_MOTOR_KV;

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.ENCODER_TO_ROTOR_RATIO;

        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Sets the angle of the pivot
     * 
     * @param angle Angle of the pivot
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle;
        angleMotor.setControl(anglePID.withPosition(angle));
    }

    /**
     * @return The current angle of the pivot
     */
    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    /**
     * @return Whether or not the pivot is on target, within
     *         PivotConstants.ANGLE_TOLERANCE
     */
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < PivotConstants.ANGLE_TOLERANCE;
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
}