package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.PivotConstants;
import frc.thunder.config.FalconConfig;

public class Pivot extends SubsystemBase {
    private TalonFX angleMotor;
    
    final PositionVoltage angleTarget = new PositionVoltage(0).withSlot(0);

    public Pivot() {
        angleMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_1, getName(), PivotConstants.PIVOT_MOTOR_INVERT, PivotConstants.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT, PivotConstants.PIVOT_MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.PIVOT_MOTOR_NEUTRAL_MODE, PivotConstants.PIVOT_MOTOR_KP, PivotConstants.PIVOT_MOTOR_KI, PivotConstants.PIVOT_MOTOR_KD, PivotConstants.PIVOT_MOTOR_KS, PivotConstants.PIVOT_MOTOR_KV);
    }

    /**
     * Sets the angle of the pivot
     * @param angle Angle of the pivot
     */
    public void setAngle(double angle) {
        angleMotor.setControl(angleTarget.withPosition(angle));
    }

    /**
     * @return The current angle of the pivot
     */
    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }
}
