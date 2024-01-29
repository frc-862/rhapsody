package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.FalconConfig;
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
            ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, ClimbConstants.CLIMB_MOTOR_KP,
            ClimbConstants.CLIMB_MOTOR_KI, ClimbConstants.CLIMB_MOTOR_KD, 
            ClimbConstants.CLIMB_MOTOR_KS, ClimbConstants.CLIMB_MOTOR_KV);
        climbMotorL = FalconConfig.createMotor(CAN.CLIMB_LEFT, CAN.CANBUS_FD,
            ClimbConstants.CLIMB_LEFT_MOTOR_INVERT, 
            ClimbConstants.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT, 
            ClimbConstants.CLIMB_MOTOR_STATOR_CURRENT_LIMIT, 
            ClimbConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE, ClimbConstants.CLIMB_MOTOR_KP,
            ClimbConstants.CLIMB_MOTOR_KI, ClimbConstants.CLIMB_MOTOR_KD, 
            ClimbConstants.CLIMB_MOTOR_KS, ClimbConstants.CLIMB_MOTOR_KV);

        LightningShuffleboard.setDoubleSupplier("Climb", "Height", () -> getHeight());
    }

    /**
     * sets power to both climb motors
     * @param power
     */
    public void setPower(double power) {
        climbMotorR.set(power);
        climbMotorL.set(power);
    }

    /**
     * stops all climb motors
     */
    public void stopClimb(){
        setPower(0);
    }

    /**
     * @return height of climb
     */
    public double getHeight(){
        return climbMotorR.getRotorPosition().getValueAsDouble(); // TODO: check if returns correct height value
    }
}
