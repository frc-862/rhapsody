package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.RhapsodyPivotConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class PivotRhapsody extends SubsystemBase implements Pivot {

    // sim pivot
    private DCMotor simGearbox = DCMotor.getFalcon500(2);
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(
        simGearbox, SingleJointedArmSim.estimateMOI(RhapsodyPivotConstants.LENGTH, RhapsodyPivotConstants.PIVOT_MASS), 
        RhapsodyPivotConstants.GEAR_RATIO), simGearbox, RhapsodyPivotConstants.GEAR_RATIO, RhapsodyPivotConstants.LENGTH, 
        Units.degreesToRadians(RhapsodyPivotConstants.MIN_ANGLE), Units.degreesToRadians(RhapsodyPivotConstants.MAX_ANGLE),
        true, Units.degreesToRadians(RhapsodyPivotConstants.STOW_ANGLE));

    private PIDController simPivotPid = new PIDController(RhapsodyPivotConstants.SIM_KP, RhapsodyPivotConstants.SIM_KI, 
        RhapsodyPivotConstants.SIM_KD);

    private Pose3d simPivotPose;

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
    private Swerve drivetrain;

    public PivotRhapsody(Swerve drivetrain) {
        this.drivetrain = drivetrain;

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

        if (!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDoubleSupplier("Pivot", "CurrentAngle", () -> (getAngle() * 360));
            LightningShuffleboard.setDoubleSupplier("Pivot", "TargetAngle", () -> targetAngle * 360);
            LightningShuffleboard.setBoolSupplier("Pivot", "OnTarget", () -> onTarget());

            LightningShuffleboard.setDoubleSupplier("Pivot", "Bias", () -> bias);
        }
    }

    @Override
    public void periodic() {

        // // SETS angle to angle of limit switch on press
        // if (getForwardLimit()) {
        // resetAngle(RhapsodyPivotConstants.MIN_ANGLE);
        // } else if (getReverseLimit()) {
        // resetAngle(RhapsodyPivotConstants.MAX_ANGLE);
        // }

        moveToTarget();

        updateLogging();
    }

    @Override
    public void simulationPeriodic() {
        // sim pivot

        simPivotPid.setIntegratorRange(-Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

        simPivotPid.setSetpoint(targetAngle * 360);

        double pivotPIDOutput = simPivotPid.calculate(getAngle() * 360);

        LightningShuffleboard.setDouble("Pivot", "SimPIDOutput", pivotPIDOutput);
        LightningShuffleboard.setDouble("Pivot", "SimIntegral", simPivotPid.getI());

        pivotSim.setInputVoltage(pivotPIDOutput * 12);
        pivotSim.update(0.01);

        simPivotPose = new Pose3d(
            drivetrain.getPose().getX() + Math.cos(drivetrain.getPose().getRotation().getRadians()) 
            * RhapsodyPivotConstants.LENGTH * Math.cos(pivotSim.getAngleRads()), 
            drivetrain.getPose().getY() + Math.sin(drivetrain.getPose().getRotation().getRadians())
            * RhapsodyPivotConstants.LENGTH * Math.cos(pivotSim.getAngleRads()),
            RhapsodyPivotConstants.LENGTH * Math.sin(pivotSim.getAngleRads()),
            new Rotation3d(0d, pivotSim.getAngleRads(), drivetrain.getPose().getRotation().getRadians()));
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
     *
     * @param angle Angle of the pivot in degrees
     */
    public void setTargetAngle(double angle) {
        targetAngle = (MathUtil.clamp(angle + bias, RhapsodyPivotConstants.MIN_ANGLE, RhapsodyPivotConstants.MAX_ANGLE)
                / 360);
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
        if (RobotBase.isSimulation()) {
            return Units.radiansToRotations(pivotSim.getAngleRads());
        }
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
     *
     * @return true if pressed
     */
    public boolean getForwardLimit() {
        return false; // angleMotor.getForwardLimit().refresh().getValue() ==
                      // ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets reverse limit switch
     *
     * @return true if pressed
     */
    public boolean getReverseLimit() {
        return false; // angleMotor.getReverseLimit().refresh().getValue() ==
                      // ReverseLimitValue.ClosedToGround;
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
