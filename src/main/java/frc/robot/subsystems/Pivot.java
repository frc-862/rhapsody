package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
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
    // private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);
    // private final MotionMagicVoltage motionMagicPID = new MotionMagicVoltage(0);
    private final PIDController angleController = new PIDController(0.06, 0, 0);
    private double bias = 0;

    private double targetAngle = PivotConstants.STOW_ANGLE;

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), PivotConstants.ENCODER_TO_MECHANISM_RATIO, 0.00401, 0.568, Math.toRadians(15), Math.toRadians(95), true, Math.toRadians(15)); //TODO: complete adding constants
    private TalonFXSimState pivotSimState;
    private CANcoderSimState encoderSimState;

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
                .withMagnetOffset(PivotConstants.ENCODER_OFFSET)
                .withSensorDirection(PivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_ANGLE_CANCODER, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        angleMotor = new ThunderBird(CAN.PIVOT_ANGLE_MOTOR, CAN.CANBUS_FD, PivotConstants.MOTOR_INVERT,
                        PivotConstants.MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.MOTOR_BRAKE_MODE);
        angleMotor.configPIDF(0, PivotConstants.MOTOR_KP, PivotConstants.MOTOR_KI,
                PivotConstants.MOTOR_KD, PivotConstants.MOTOR_KS, PivotConstants.MOTOR_KV);
        TalonFXConfiguration motorConfig = angleMotor.getConfig();

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.ROTOR_TO_ENCODER_RATIO;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.MAGIC_CRUISE_VEL;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAGIC_ACCEL;
        motionMagicConfigs.MotionMagicJerk = PivotConstants.MAGIC_JERK;

        angleMotor.applyConfig(motorConfig);
        
        pivotTuner = new FalconTuner(angleMotor, "Pivot", this::setTargetAngle, 0d);
        if(Robot.isSimulation()) {
            pivotSimState = angleMotor.getSimState();
            encoderSimState = angleEncoder.getSimState();

            encoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        }
    }

    @Override
    public void simulationPeriodic() {
        pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
        pivotSim.update(0.02);

        // pivotSimState.setRawRotorPosition((pivotSim.getAngleRads() / (2 * Math.PI)) / PivotConstants.ROTOR_TO_ENCODER_RATIO);
        encoderSimState.setRawPosition(pivotSim.getAngleRads() / (2 * Math.PI));


        pivotMech.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
        LightningShuffleboard.set("sim", "pivot", mech2d);
        LightningShuffleboard.setDouble("Pivot", "simmm", Math.toDegrees(pivotSim.getAngleRads()));

        LightningShuffleboard.setDouble("Pivot", "voltage", pivotSimState.getMotorVoltage());
    }

    @Override
    public void periodic() {
        pivotTuner.update();
        LightningShuffleboard.setDouble("Pivot", "Target Angle", targetAngle);
        LightningShuffleboard.setDouble("Pivot", "Bias", bias);
        LightningShuffleboard.setBool("Pivot", "On Target", onTarget());
        LightningShuffleboard.setDouble("Pivot", "Angle", getAngle());
    }

    /**
     * Sets the angle of the pivot
     * 
     * @param angle Angle of the pivot in degrees
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle;
        angleMotor.setControl(anglePID.withPosition(angle / 360));
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
