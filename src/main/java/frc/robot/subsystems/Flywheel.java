package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.FlywheelConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.thunder.config.FalconConfig;

public class Flywheel extends SubsystemBase {
    private TalonFX shooterTopMotor; // TODO figure out which is top vs bottom
    private TalonFX shooterBottomMotor;

    private final VelocityVoltage rpmPID = new VelocityVoltage(0).withSlot(0);
    private double targetRPM = 0;
    private double bias = 0;

    // new flywheel simulation
    private final DCMotor m_flywheelGearbox = DCMotor.getFalcon500(2);
    private final FlywheelSim flywheelsim = new FlywheelSim(m_flywheelGearbox, FlywheelConstants.FLYWHEEL_GEARING, bias); // TODO: get inertia value
    public Flywheel() {
        shooterTopMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_1, CAN.CANBUS_FD,
                FlywheelConstants.FLYWHEEL_MOTOR_1_INVERT,
                FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP,
                FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
                FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);
        shooterBottomMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_2, CAN.CANBUS_FD,
                FlywheelConstants.FLYWHEEL_MOTOR_2_INVERT,
                FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT,
                FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP,
                FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
                FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);

        
    }

    public void simulationPeriodic() {
        flywheelsim.setInput(targetRPM);
        flywheelsim.update(0.02); 
    }

    /**
     * Sets the RPM of all flywheel motors
     * 
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        targetRPM = rpm;
        shooterTopMotor.setControl(rpmPID.withVelocity(rpm));
        shooterBottomMotor.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 1
     * 
     * @param rpm RPM of the flywheel
     */
    public void setTopMoterRPM(double rpm) {
        targetRPM = rpm;
        shooterTopMotor.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 2
     * 
     * @param rpm RPM of the flywheel
     */
    public void setBottomMoterRPM(double rpm) {
        targetRPM = rpm;
        shooterBottomMotor.setControl(rpmPID.withVelocity(rpm));
    }

    /**
     * @return The current RPM of flywheel # 1
     */
    public double getTopMotorRPM() {
        return (shooterTopMotor.getVelocity().getValue());
    }

    /**
     * @return The current RPM of flywheel # 2
     */
    public double getBottomMotorRPM() {
        return (shooterBottomMotor.getVelocity().getValue());
    }

    /**
     * @return Whether or not flywheel # 1 is on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean topMotorRPMOnTarget() {
        return Math.abs(getTopMotorRPM() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not flywheel # 2 is on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean bottomMotorRPMOnTarget() {
        return Math.abs(getBottomMotorRPM() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
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