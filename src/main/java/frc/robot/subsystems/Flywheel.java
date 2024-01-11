package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.FlywheelConstants;
import frc.thunder.config.FalconConfig;

public class Flywheel extends SubsystemBase {
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    
    final VelocityVoltage rpmTarget = new VelocityVoltage(0).withSlot(0);
    
    public Flywheel() {
        shooterMotor1 = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_1, getName(), FlywheelConstants.FLYWHEEL_MOTOR_1_INVERT, FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP, FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD, FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);
        shooterMotor2 = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_2, getName(), FlywheelConstants.FLYWHEEL_MOTOR_2_INVERT, FlywheelConstants.FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, FlywheelConstants.FLYWHEEL_MOTOR_KP, FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD, FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);
    }

    /**
     * Sets the RPM of the flywheel
     * @param rpm RPM of the flywheel
     */
    public void setRPM(double rpm){
        shooterMotor1.setControl(rpmTarget.withVelocity(rpm));
        shooterMotor2.setControl(rpmTarget.withVelocity(rpm));
    }

    /**
     * @return The current RPM of the flywheel
     */
    public double getFlywheelRPM() {
        return (shooterMotor1.getVelocity().getValue() + shooterMotor2.getVelocity().getValue()) / 2;
    }
}
