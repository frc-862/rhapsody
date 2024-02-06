package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.FlywheelConstants;
import frc.thunder.config.FalconConfig;
import frc.thunder.tuning.FalconTuner;

public class Flywheel extends SubsystemBase {
    private TalonFX shooterMotor1; //TODO figure out which is top vs bottom
    private TalonFX shooterMotor2;

    private final VelocityVoltage rpmPID = new VelocityVoltage(0).withSlot(0);
    private double targetRPM = 0;

    public Flywheel() {
        shooterMotor1 = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_1, CAN.CANBUS_FD,
                FlywheelConstants.FLYWHEEL_MOTOR_1_INVERT,
                FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP,
                FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
                FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);
        shooterMotor2 = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_2, CAN.CANBUS_FD,
                FlywheelConstants.FLYWHEEL_MOTOR_2_INVERT,
                FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP,
                FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
                FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);
    }

    /**
     * Sets the RPM of all flywheel motors
     * 
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        targetRPM = rpm;
        shooterMotor1.setControl(rpmPID.withVelocity(rpm));
        shooterMotor2.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 1
     * 
     * @param rpm RPM of the flywheel
     */
    public void setMoter1RPM(double rpm) {
        targetRPM = rpm;
        shooterMotor1.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 2
     * 
     * @param rpm RPM of the flywheel
     */
    public void setMoter2RPM(double rpm) {
        targetRPM = rpm;
        shooterMotor2.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * @return The current Average RPM of the flywheel
     */
    public double getAverageMotorRPM() {
        return (shooterMotor1.getVelocity().getValue() + shooterMotor2.getVelocity().getValue()) / 2;
    }

    /**
     * @return The current RPM of flywheel # 1
     */
    public double getMotor1RPM() {
        return (shooterMotor1.getVelocity().getValue());
    }

    /**
     * @return The current RPM of flywheel # 2
     */
    public double getMotor2RPM() {
        return (shooterMotor2.getVelocity().getValue());
    }

    /**
     * @return Whether or not the flywheel is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean averageMotorRPMOnTarget() {
        return Math.abs(getAverageMotorRPM() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not flywheel # 1 is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean motor1RPMOnTarget() {
        return Math.abs(getMotor1RPM() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not flywheel # 2 is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean motor2RPMOnTarget() {
        return Math.abs(getMotor2RPM() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not all flywheel motors are on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean allMotorsOnTarget() {
        return (motor1RPMOnTarget() && motor2RPMOnTarget());
    }
}
