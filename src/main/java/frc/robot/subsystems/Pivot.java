package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    private double bias = 0;

    private double targetAngle = 0; // TODO: find initial target angle

    private FalconTuner pivotTuner;

    public Pivot() { 
        
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(PivotConstants.ENCODER_OFFSET).withSensorDirection(PivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        angleMotor = new ThunderBird(CAN.PIVOT_ANGLE_MOTOR, CAN.CANBUS_FD, PivotConstants.PIVOT_MOTOR_INVERT, PivotConstants.PIVOT_MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.PIVOT_MOTOR_BRAKE_MODE);
        angleMotor.configPIDF(0, PivotConstants.PIVOT_MOTOR_KP, PivotConstants.PIVOT_MOTOR_KI, PivotConstants.PIVOT_MOTOR_KD, PivotConstants.PIVOT_MOTOR_KS, PivotConstants.PIVOT_MOTOR_KV);
        TalonFXConfiguration motorConfig = angleMotor.getConfig();

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.ROTOR_TO_ENCODER_RATIO;

        angleMotor.applyConfig(motorConfig);

        pivotTuner = new FalconTuner(angleMotor, "Pivot", this::setTargetAngle, targetAngle);
    }

    @Override
    public void periodic() {
        pivotTuner.update();
        LightningShuffleboard.setDouble("Pivot", "Angle", getAngle());
        LightningShuffleboard.setDouble("Pivot", "CANCoder angle", angleEncoder.getPosition().getValueAsDouble());
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