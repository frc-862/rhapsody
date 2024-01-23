package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.PivotConstants;
import frc.thunder.config.FalconConfig;

public class Pivot extends SubsystemBase {
    private TalonFX angleMotor;
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);

    // TODO: find initial target angle
    private double targetAngle = 0;

    public Pivot() {
        angleMotor = FalconConfig.createMotor(CAN.FLYWHEEL_MOTOR_1, CAN.CANBUS,
                PivotConstants.PIVOT_MOTOR_INVERT, 
				PivotConstants.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT,
                PivotConstants.PIVOT_MOTOR_STATOR_CURRENT_LIMIT,
                PivotConstants.PIVOT_MOTOR_NEUTRAL_MODE, PivotConstants.PIVOT_MOTOR_KP,
                PivotConstants.PIVOT_MOTOR_KI, PivotConstants.PIVOT_MOTOR_KD,
                PivotConstants.PIVOT_MOTOR_KS, PivotConstants.PIVOT_MOTOR_KV);
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
     * @return Whether or not the pivot is on target, within PivotConstants.ANGLE_TOLERANCE
     */
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < PivotConstants.ANGLE_TOLERANCE;
    }
}
