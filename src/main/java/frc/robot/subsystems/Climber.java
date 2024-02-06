package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMBER_STATES;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.FalconConfig;
import frc.thunder.math.Conversions;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Climber extends SubsystemBase {
    // create variables
    private TalonFX climbMotorR;
    private TalonFX climbMotorL;

    private PositionTorqueCurrentFOC setPointR;
    private PositionTorqueCurrentFOC setPointL;

    private CLIMBER_STATES state = CLIMBER_STATES.STOW;
    private Swerve drivetrain;

    private boolean hasTipped = false;
    private boolean hasStowed = false;
    private boolean hasGroundedR = false;
    private boolean hasGroundedL = false;

    public Climber(Swerve drivetrain) {
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
        LightningShuffleboard.setDoubleSupplier("Climb", "Left Setpoint", () -> getSetpointL());
        LightningShuffleboard.setDoubleSupplier("Climb", "Right Setpoint", () -> getSetpointR());
        LightningShuffleboard.set("Climb", "Left Lower Pose", convertLowerPose(getHeightL(), false));
        LightningShuffleboard.set("Climb", "Right Lower Pose", convertLowerPose(getHeightR(), true));
        LightningShuffleboard.set("Climb", "Left Upper Pose", convertUpperPose(getHeightL(), false));
        LightningShuffleboard.set("Climb", "Right Upper Pose", convertUpperPose(getHeightR(), true));
        LightningShuffleboard.set("Climb", "Left Lower Setpoint", convertLowerPose(getSetpointL(), false));
        LightningShuffleboard.set("Climb", "Right Lower Setpoint", convertLowerPose(getSetpointR(), true));
        LightningShuffleboard.set("Climb", "Left Upper Setpoint", convertUpperPose(getSetpointL(), false));
        LightningShuffleboard.set("Climb", "Right Upper Setpoint", convertUpperPose(getSetpointR(), true));
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

    public void setPowerL(double powerL) {
        climbMotorL.set(powerL);
    }

    public void setPowerR(double powerR) {
        climbMotorL.set(powerR);
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
     * @param leftInches setpoint for left climb motor in inches
     * @param rightInches setpoint for right climb motor in inches
     */
    public void setSetpoint(double leftInches, double rightInches){
        this.setPointL = new PositionTorqueCurrentFOC(Conversions.getInputShaftRotations(leftInches/ClimbConstants.WINCH_CIRCUFERENCE, ClimbConstants.GEAR_REDUCTION));
        this.setPointR = new PositionTorqueCurrentFOC(Conversions.getInputShaftRotations(rightInches/ClimbConstants.WINCH_CIRCUFERENCE, ClimbConstants.GEAR_REDUCTION));

        climbMotorL.setControl(this.setPointL);
        climbMotorR.setControl(this.setPointR);
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
    public void stop(){
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

    /**
     * @return the setpoint of the right climb arm
     */
    public double getSetpointR(){
        return Conversions.getOutputShaftRotations(this.setPointR.Position, ClimbConstants.GEAR_REDUCTION) * ClimbConstants.WINCH_CIRCUFERENCE;
    }

    /**
     * @return the setpoint of the left climb arm
     */
    public double getSetpointL(){
        return Conversions.getOutputShaftRotations(this.setPointL.Position, ClimbConstants.GEAR_REDUCTION) * ClimbConstants.WINCH_CIRCUFERENCE;
    }

    /**
     * @param height the height to convert to a pose, in inches
     * @param isRight whether the pose is for the right arm
     * @return the pose of the climb arm
     */
    private Pose3d convertLowerPose(double height, boolean isRight) {
        // law of cosines to get angle of lower arm
        double pitch = (Math.PI/2)-Math.acos(
            (-Math.pow(ClimbConstants.UPPER_LENGTH, 2)+Math.pow(height, 2)+Math.pow(ClimbConstants.LOWER_LENGTH, 2))/
            (2*height*ClimbConstants.LOWER_LENGTH));
        if (isRight) {
            return ClimbConstants.LOWER_OFFSET.plus(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, pitch, 0)).plus(
                        ClimbConstants.LEFT_RIGHT_OFFSET));
        } else {
            return ClimbConstants.LOWER_OFFSET.plus(
            new Transform3d(
                new Translation3d(0, 0, -Units.inchesToMeters(height)),
                new Rotation3d(0, pitch, 0)).plus(
                    ClimbConstants.LEFT_RIGHT_OFFSET.inverse())
                );
        }
    }

    /**
     * @param height the height to convert to a pose, in inches
     * @param isRight whether the pose is for the right arm
     * @return the pose of the climb arm
     */
    private Pose3d convertUpperPose(double height, boolean isRight) {
        Pose3d lowerPose = convertLowerPose(height, isRight);
        double lowerPitch = lowerPose.getRotation().getY();
        // law of cosines to get angle of upper arm
        double pitch = Math.PI-Math.acos(
            (-Math.pow(height, 2)+Math.pow(ClimbConstants.LOWER_LENGTH, 2)+Math.pow(ClimbConstants.UPPER_LENGTH, 2))/
            (2*ClimbConstants.LOWER_LENGTH*ClimbConstants.UPPER_LENGTH)) - lowerPitch;
        // simple trig to get height of upper arm (equal to height of end of lower arm)
        double poseZ = Units.inchesToMeters(ClimbConstants.LOWER_LENGTH*Math.sin(lowerPitch)) + lowerPose.getTranslation().getZ();
        // simple trig to get distance from end of lower arm to base of upper arm
        double poseX = Units.inchesToMeters(ClimbConstants.LOWER_LENGTH*Math.cos(lowerPitch)) + lowerPose.getTranslation().getX();
        if (isRight) {
            return ClimbConstants.UPPER_OFFSET.plus(
                new Transform3d(
                    new Translation3d(poseX, 0, poseZ),
                    new Rotation3d(0, pitch, 0)).plus(
                        ClimbConstants.LEFT_RIGHT_OFFSET)
                    );
        } else {
            return ClimbConstants.UPPER_OFFSET.plus(
                new Transform3d(
                    new Translation3d(poseX, 0, poseZ),
                    new Rotation3d(0, pitch, 0)).plus(
                        ClimbConstants.LEFT_RIGHT_OFFSET.inverse())
                );
        }
    }

    /**
     * stores state value for climber
     * @param state
     */
    public void setState(CLIMBER_STATES state){
        this.state = state;
    }

    /**
     * @return current stored state of climber
     */
    public CLIMBER_STATES getState() {
        return state;
    }

    /**
     * tell climber if climber has begun climbing
     * @param hasTipped
     */
    public void setHasTipped(boolean hasTipped){
        this.hasTipped = hasTipped;
    }

    /**
     * tell climber if the right arm has been fully extended (robot should have returned to ground after climb)
     * @param hasGroundedR
     */
    public void setHasGroundedR(boolean hasGroundedR){
        this.hasGroundedR = hasGroundedR;
    }

    /**
     * tell climber if the left arm has been fully extended (robot should have returned to ground after climb)
     * @param hasGroundedL
     */
    public void setHasGroundedL(boolean hasGroundedL){
        this.hasGroundedL = hasGroundedL;
    }

    /**
     * tell climber if both arms have been retracted after returning to gound
     * @param hasStowed
     */
    public void setHasStowed(boolean hasStowed){
        this.hasStowed = hasStowed;
    }

    /**
     * resets has stowed/grounded/tipped values
     */
    public void resetHasValues() {
        hasGroundedL = false;
        hasGroundedR = false;
        hasStowed = false;
        hasTipped = false;
    }

    @Override
    public void periodic() {
        // updates height based on limit switches
        for (TalonFX motor : new TalonFX[] {climbMotorR, climbMotorL}) {
            if (motor.getRotorPosition().getValueAsDouble() > ClimbConstants.MAX_HEIGHT) {
                motor.setPosition(ClimbConstants.MAX_HEIGHT);
            }
            if (motor.getRotorPosition().getValueAsDouble() < 0 || motor.getReverseLimit().getValueAsDouble() == 0) {
                motor.setPosition(0);
            }
        }
        
        // updates states
        if (hasTipped && getHeightL() < ClimbConstants.MAX_HEIGHT/2 &&
			getHeightR() < ClimbConstants.MAX_HEIGHT/2){
            state = CLIMBER_STATES.CLIMBED;
            resetHasValues();
        }

        if (getHeightR() <= ClimbConstants.CLIMB_RETRACTION_TOLERANCE && 
        getHeightL() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE && !drivetrain.isTipped() && hasStowed) {
            state = CLIMBER_STATES.STOW;
            resetHasValues();
        }

        if (ClimbConstants.MAX_HEIGHT - getHeightR() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE &&
        ClimbConstants.MAX_HEIGHT - getHeightL() <= ClimbConstants.CLIMB_EXTENSION_TOLERANCE &&
        !drivetrain.isTipped() && hasGroundedR && hasGroundedL) {
            state = CLIMBER_STATES.GROUNDED;
            resetHasValues();
        }
    }
}
