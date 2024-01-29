package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.FalconConfig;
import frc.thunder.math.Conversions;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Climber extends SubsystemBase {
    // create variables
    public TalonFX climbMotorR;
    public TalonFX climbMotorL;
    public double setPoint;

    public Climber() {
        // configure climb motors
        climbMotorR = FalconConfig.createMotor(CAN.CLIMB_RIGHT, CAN.CANBUS_FD,
            ClimbConstants.CLIMB_RIGHT_MOTOR_INVERT, 
            ClimbConstants.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT, 
            ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, 
            ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE,
            ClimbConstants.EXTEND_KP,
            ClimbConstants.EXTEND_KI,
            ClimbConstants.EXTEND_KD,
            ClimbConstants.RETRACT_KP,
            ClimbConstants.RETRACT_KI,
            ClimbConstants.RETRACT_KD);
        climbMotorL = FalconConfig.createMotor(CAN.CLIMB_LEFT, CAN.CANBUS_FD,
            ClimbConstants.CLIMB_LEFT_MOTOR_INVERT, 
            ClimbConstants.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT, 
            ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, 
            ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE,
            ClimbConstants.EXTEND_KP,
            ClimbConstants.EXTEND_KI,
            ClimbConstants.EXTEND_KD,
            ClimbConstants.RETRACT_KP,
            ClimbConstants.RETRACT_KI,
            ClimbConstants.RETRACT_KD);

        LightningShuffleboard.setDoubleSupplier("Climb", "Left Height", () -> getHeightL());
        LightningShuffleboard.setDoubleSupplier("Climb", "Right Height", () -> getHeightR());
    }

    /**
     * sets power to climb motors
     * @param powerR the power to set the right climb motor to
     * @param powerL the power to set the left climb motor to
     */
    public void setPower(double powerL, double powerR) {
        climbMotorR.set(powerR);
        climbMotorL.set(powerL);
    }

    /**
     * sets power to both climb motors
     * @param power the power to set both climb motors to
     */
    public void setPower(double power) {
        setPower(power, power);
    }

    /**
     * sets the setpoint of the climb motors
     * @param setPointL setpoint for left climb motor in inches
     * @param setPointR setpoint for right climb motor in inches
     */
    public void setSetpoint(double setPointL, double setPointR){
        setPointL = Conversions.getInputShaftRotations(setPointL/ClimbConstants.WINCH_CIRCUFERENCE, ClimbConstants.GEAR_REDUCTION);
        setPointR = Conversions.getInputShaftRotations(setPointR/ClimbConstants.WINCH_CIRCUFERENCE, ClimbConstants.GEAR_REDUCTION);
        climbMotorL.setControl(new PositionTorqueCurrentFOC(setPointL));
        climbMotorR.setControl(new PositionTorqueCurrentFOC(setPointR));
    }

    /**
     * sets the setpoint of both climb motors
     * @param setPoint setpoint for both climb motors in inches
     */
    public void setSetpoint(double setPoint){
        setSetpoint(setPoint, setPoint);
    }

    /**
     * stops all climb motors
     */
    public void stopClimb(){
        setPower(0d);
    }

    /**
     * @return height of right climb arm
     */
    public double getHeightR(){
        return Conversions.getOutputShaftRotations(climbMotorR.getRotorPosition().getValueAsDouble(), ClimbConstants.GEAR_REDUCTION) * ClimbConstants.WINCH_CIRCUFERENCE;
    }

    /**
     * @return height of left climb arm
     */
    public double getHeightL(){
        return Conversions.getOutputShaftRotations(climbMotorL.getRotorPosition().getValueAsDouble(), ClimbConstants.GEAR_REDUCTION) * ClimbConstants.WINCH_CIRCUFERENCE;
    }

    @Override
    public void periodic() {
        for (TalonFX motor : new TalonFX[] {climbMotorR, climbMotorL}) {
            if (motor.getRotorPosition().getValueAsDouble() > ClimbConstants.MAX_HEIGHT) {
                motor.setPosition(ClimbConstants.MAX_HEIGHT);
            }
            if (motor.getRotorPosition().getValueAsDouble() < 0 || motor.getReverseLimit().getValueAsDouble() == 0) {
                motor.setPosition(0);
            }
        }
    }
}
