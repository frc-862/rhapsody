package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;
import frc.robot.Robot;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private ThunderBird angleMotor;
    private CANcoder angleEncoder;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    private double bias = 0;

    private double targetAngle = 0; // TODO: find initial target angle

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), PivotConstants.ENCODER_TO_MECHANISM_RATIO, 0.00401, 0.568, Math.toRadians(15), Math.toRadians(95), false, Math.toRadians(15)); //TODO: complete adding constants
    private TalonFXSimState pivotSimState;

    private final Mechanism2d mech2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d root = mech2d.getRoot("Pivot Root", 13, 5);
    private final MechanismLigament2d pivotMech =
            root.append(
                new MechanismLigament2d(
                    "Pivot",
                    30,
                    Units.radiansToDegrees(0),
                    6,
                    new Color8Bit(Color.kYellow)));

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
        motorConfig.Feedback.RotorToSensorRatio = 1/PivotConstants.ROTOR_TO_ENCODER_RATIO;

        angleMotor.applyConfig(motorConfig);

        if(Robot.isSimulation()) {
            pivotSimState = angleMotor.getSimState();
            pivotTuner = new FalconTuner(angleMotor, "Pivot", this::setTargetAngle, 15d);
        }
    }

    @Override
    public void simulationPeriodic() {
        pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
        pivotSim.update(0.02);

        pivotSimState.setRawRotorPosition(pivotSim.getAngleRads() * (2 * Math.PI));

        pivotMech.setAngle(getAngle());
        LightningShuffleboard.set("sim", "pivot", mech2d);
        pivotTuner.update();

        LightningShuffleboard.setDouble("Pivot", "voltage", pivotSimState.getMotorVoltage());
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Pivot", "Angle", getAngle());
        LightningShuffleboard.setDouble("Pivot", "Target Angle", targetAngle);
        LightningShuffleboard.setDouble("Pivot", "Bias", bias);
        LightningShuffleboard.setBool("Pivot", "On Target", onTarget());
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