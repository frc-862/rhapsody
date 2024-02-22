package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.PivotConstants;
import frc.thunder.config.FalconConfig;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;

public class Flywheel extends SubsystemBase {
    // private ThunderBird topMotor; // TODO figure out which is top vs bottom
    // private ThunderBird bottomMotor;
    private TalonFX topMotor;
    private TalonFX bottomMotor;

    private final PIDController topRPMPID;
    private final PIDController bottomRPMPID;
    private final SimpleMotorFeedforward topFeedForward;
    private final SimpleMotorFeedforward bottomFeedForward;

    // private final VelocityVoltage rpmPID = new VelocityVoltage(0, 0, false, FlywheelConstants.MOTOR_KV, 
    // 0, false,false, false);
    // private final VelocityVoltage rpmPID = new VelocityVoltage(0d).withSlot(0);
    private double topTargetRPM = 0;
    private double bottomTargetRPM = 0;
    private double bias = 0;
    private boolean tuning = false;
    
    // private FalconTuner topTuner;
    // private FalconTuner bottomTuner;
    
    public Flywheel() {
        /* TEST after kettering basic stuff for now */
        // topMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD,
        // FlywheelConstants.MOTOR_TOP_INVERT, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
        // FlywheelConstants.MOTOR_BRAKE_MODE);
        // bottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD,
        // FlywheelConstants.MOTOR_BOTTOM_INVERT, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
        // FlywheelConstants.MOTOR_BRAKE_MODE);
        // topMotor.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI,
        //         FlywheelConstants.MOTOR_KD, FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV);
        // bottomMotor.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI,
        //         FlywheelConstants.MOTOR_KD, FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV);

        // note - if you wanna tune these together, you should be able to set the tab to the same
        // name and it'll read the same values for both instances
        // topTuner = new FalconTuner(shooterTopMotor, "Flywheel Top", this::setTopMoterRPM,
        //         topTargetRPM);
        // bottomTuner = new FalconTuner(shooterBottomMotor, "Flywheel Bottom",
        //         this::setBottomMoterRPM, bottomTargetRPM);

        /* TEMP base code */
        topMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD,
            FlywheelConstants.MOTOR_TOP_INVERT, 0, 0, NeutralModeValue.Coast);
        bottomMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD,
            FlywheelConstants.MOTOR_BOTTOM_INVERT, 0, 0, NeutralModeValue.Coast);
        
        topRPMPID = new PIDController(FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI, FlywheelConstants.MOTOR_KD);
        bottomRPMPID = new PIDController(FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI, FlywheelConstants.MOTOR_KD);

        topFeedForward = new SimpleMotorFeedforward(FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV, FlywheelConstants.MOTOR_KA);
        bottomFeedForward = new SimpleMotorFeedforward(FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV, FlywheelConstants.MOTOR_KA);

        topRPMPID.setTolerance(FlywheelConstants.RPM_TOLERANCE);
        bottomRPMPID.setTolerance(FlywheelConstants.RPM_TOLERANCE);

        initLogging();
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Top RPM", this::getTopMotorRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Bottom RPM", this::getBottomMotorRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Target Top RPM", () -> topTargetRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Target Bottom RPM", () -> bottomTargetRPM);

        LightningShuffleboard.setBoolSupplier("Flywheel", "Top on Target", () -> topMotorRPMOnTarget());
        LightningShuffleboard.setBoolSupplier("Flywheel", "Bottom on Target", () -> bottomMotorRPMOnTarget());

        LightningShuffleboard.setDoubleSupplier("Flywheel", "Top Power", () -> topMotor.get());
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Bottom Power", () -> bottomMotor.get());

        LightningShuffleboard.setDoubleSupplier("Flywheel", "Bias", this::getBias);
    }
    
    @Override
    public void periodic() {
        tuning = LightningShuffleboard.getBool("Flywheel", "Tuning", false);
        // topTuner.update();
        // bottomTuner.update();
        
        topMotor.set(topRPMPID.calculate(getTopMotorRPM()) + topFeedForward.calculate(topTargetRPM));
        bottomMotor.set(bottomRPMPID.calculate(getBottomMotorRPM()) + bottomFeedForward.calculate(bottomTargetRPM));

        LightningShuffleboard.setDouble("Flywheel", "top PID output", topRPMPID.calculate(getTopMotorRPM()));
        LightningShuffleboard.setDouble("Flywheel", "bottom PID output", bottomRPMPID.calculate(getBottomMotorRPM()));
        LightningShuffleboard.setDouble("Flywheel", "top Feed forward", topFeedForward.calculate(topTargetRPM));
        LightningShuffleboard.setDouble("Flywheel", "bottom Feed forward", bottomFeedForward.calculate(bottomTargetRPM));



        if (tuning) {
            topRPMPID.setP(LightningShuffleboard.getDouble("Flywheel", "Top P", topRPMPID.getP()));
            topRPMPID.setI(LightningShuffleboard.getDouble("Flywheel", "Top I", topRPMPID.getI()));
            topRPMPID.setD(LightningShuffleboard.getDouble("Flywheel", "Top D", topRPMPID.getD()));

            bottomRPMPID.setP(LightningShuffleboard.getDouble("Flywheel", "Bottom P", bottomRPMPID.getP()));
            bottomRPMPID.setI(LightningShuffleboard.getDouble("Flywheel", "Bottom I", bottomRPMPID.getI()));
            bottomRPMPID.setD(LightningShuffleboard.getDouble("Flywheel", "Bottom D", bottomRPMPID.getD()));

            setTopMoterRPM(LightningShuffleboard.getDouble("Flywheel", "Top Target RPM", topTargetRPM));
            setBottomMoterRPM(LightningShuffleboard.getDouble("Flywheel", "Bottom Target RPM", bottomTargetRPM));
        }
    }

    // public void setPower() {
    //     shooterBottomMotor.set(.9);
    //     shooterTopMotor.set(.9);
    // }

    /**
     * Sets the RPM of all flywheel motors
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        topTargetRPM = rpm;
        bottomTargetRPM = rpm;
        
        topRPMPID.setSetpoint(rpm);
        bottomRPMPID.setSetpoint(rpm);
        
        // double rps = rpm / 60;
        // topMotor.setControl(
        //         rpmPID.withVelocity(rps).withFeedForward(rps * FlywheelConstants.MOTOR_KV));
        // bottomMotor.setControl(
        //         rpmPID.withVelocity(rps).withFeedForward(rps * FlywheelConstants.MOTOR_KV));
    }

    /**
     * Sets the RPM of top flywheel
     * @param rpm RPM of the flywheel
     */
    public void setTopMoterRPM(double rpm) {
        topTargetRPM = rpm;
        topRPMPID.setSetpoint(rpm);

        // double rps = rpm / 60;
        // topMotor.setControl(
        //         rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
    }

    /**
     * Sets the RPM of bottom flywheel
     * @param rpm RPM of the flywheel
     */
    public void setBottomMoterRPM(double rpm) {
        bottomTargetRPM = rpm;
        bottomRPMPID.setSetpoint(rpm);
        
        // double rps = rpm / 60;
        // bottomMotor.setControl(
        //         rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
    }

    /**
     * @return The current RPM of flywheel top
     */
    public double getTopMotorRPM() {
        return (topMotor.getVelocity().getValue() * 60);
    }

    /**
     * @return The current RPM of flywheel bottom
     */
    public double getBottomMotorRPM() {
        return (bottomMotor.getVelocity().getValue() * 60);
    }

    /**
     * @return Whether or not flywheel # 1 is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean topMotorRPMOnTarget() {
        return Math.abs(getTopMotorRPM() - topTargetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not flywheel # 2 is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean bottomMotorRPMOnTarget() {
        return Math.abs(getBottomMotorRPM() - bottomTargetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not all flywheel motors are on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean allMotorsOnTarget() {
        return (topMotorRPMOnTarget() && bottomMotorRPMOnTarget());
    }

    /**
     * Sets the voltage to a small amount so the flywheel coasts to a stop
     */
    public void coast() {
        bottomMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
        topMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
    }

    /**
     * @return The bias to add to the target RPM of the flywheel
     */
    public double getBias() {
        return bias;
    }

    /**
     * Increases the bias of the Flywheel by set amount
     */
    public void increaseBias() {
        bias += FlywheelConstants.BIAS_INCREMENT;
    }

    /**
     * Decreases the bias of the Flywheel by set amount
     */
    public void decreaseBias() {
        bias -= FlywheelConstants.BIAS_INCREMENT;
    }

    /**
     * Resets the bias of the Flywheel
     */
    public void resetBias() {
        bias = 0;
    }
}
