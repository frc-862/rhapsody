package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.FlywheelConstants;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;

public class Flywheel extends SubsystemBase {
    private ThunderBird shooterTopMotor; // TODO figure out which is top vs bottom
    private ThunderBird shooterBottomMotor;

    // private final VelocityVoltage rpmPID = new VelocityVoltage(0, 0, false, feed, 0, fasle,
    // false, false);
    private final VelocityVoltage rpmPID = new VelocityVoltage(0d).withSlot(0);
    private double topTargetRPM = 0;
    private double bottomTargetRPM = 0;
    private double bias = 0;

    private FalconTuner topTuner;
    private FalconTuner bottomTuner;

    public Flywheel() {
        shooterTopMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD,
                FlywheelConstants.MOTOR_TOP_INVERT, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.MOTOR_BRAKE_MODE);
        shooterBottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD,
                FlywheelConstants.MOTOR_BOTTOM_INVERT, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.MOTOR_BRAKE_MODE);

        shooterTopMotor.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI,
                FlywheelConstants.MOTOR_KD, FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV);

        shooterBottomMotor.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI,
                FlywheelConstants.MOTOR_KD, FlywheelConstants.MOTOR_KS, FlywheelConstants.MOTOR_KV);

        // note - if you wanna tune these together, you should be able to set the tab to the same
        // name and it'll read the same values for both instances
        topTuner = new FalconTuner(shooterTopMotor, "Flywheel Top", this::setTopMoterRPM,
                topTargetRPM);
        bottomTuner = new FalconTuner(shooterBottomMotor, "Flywheel Bottom",
                this::setBottomMoterRPM, bottomTargetRPM);

        initLogging();
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Top RPM", this::getTopMotorRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Bottom RPM", this::getBottomMotorRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Target Top RPM", () -> topTargetRPM);
        LightningShuffleboard.setDoubleSupplier("Flywheel", "Target Bottom RPM",
                () -> bottomTargetRPM);

        LightningShuffleboard.setBoolSupplier("Flywheel", "Top on Target",
                () -> topMotorRPMOnTarget());
        LightningShuffleboard.setBoolSupplier("Flywheel", "Bottom on Target",
                () -> bottomMotorRPMOnTarget());

        LightningShuffleboard.setDoubleSupplier("Flywheel", "Bias", this::getBias);
    }

    @Override
    public void periodic() {
        topTuner.update();
        bottomTuner.update();

    }

    public void setPower() {
        shooterBottomMotor.set(.9);
        shooterTopMotor.set(.9);
    }

    /**
     * Sets the RPM of all flywheel motors
     * 
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        topTargetRPM = rpm;
        bottomTargetRPM = rpm;

        double rps = rpm / 60;

        shooterTopMotor.setControl(
                rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
        shooterBottomMotor.setControl(
                rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
    }

    /**
     * Sets the RPM of flywheel # 1
     * 
     * @param rpm RPM of the flywheel
     */
    public void setTopMoterRPM(double rpm) {
        topTargetRPM = rpm;
        double rps = rpm / 60;

        shooterTopMotor.setControl(
                rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
    }

    /**
     * Sets the RPM of flywheel # 2
     * 
     * @param rpm RPM of the flywheel
     */
    public void setBottomMoterRPM(double rpm) {
        bottomTargetRPM = rpm;
        double rps = rpm / 60;
        shooterBottomMotor.setControl(
                rpmPID.withVelocity(rps).withFeedForward(FlywheelConstants.MOTOR_KV * rps));
    }

    /**
     * @return The current RPM of flywheel top
     */
    public double getTopMotorRPM() {
        return (shooterTopMotor.getVelocity().getValue() * 60);
    }

    /**
     * @return The current RPM of flywheel bottom
     */
    public double getBottomMotorRPM() {
        return (shooterBottomMotor.getVelocity().getValue() * 60);
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

    public void coast() {
        shooterBottomMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
        shooterTopMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
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
