package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Pivot extends Subsystem {

    /**
     * Sets the target angle of the pivot
     * 
     * @param angle Angle of the pivot
     */
    void setTargetAngle(double angle);

    void setPower(double power);

    /**
     * @return The current angle of the pivot in degrees
     */
    double getAngle();

    /**
     * @return Whether or not the pivot is on target, within Angle tolerance
     */
    boolean onTarget();

    /**
     * Gets forward limit switch
     * 
     * @return true if pressed
     */
    boolean getForwardLimit();

    /**
     * Gets reverse limit switch
     * 
     * @return true if pressed
     */
    boolean getReverseLimit();

    /**
     * @return The bias to add to the target angle of the pivot
     */
    double getBias();

    /**
     * Increases the bias of the pivot by set amount
     */
    void increaseBias();

    /**
     * Decreases the bias of the pivot by set amount
     */
    void decreaseBias();

    /**
     * Resets the bias of the pivot
     */
    void resetBias();

    /**
     * @param angle angle to set the pivot angle to
     */
    void resetAngle(double angle);

    /**
     * @return current stow angle
     */
    double getStowAngle();

    /**
     * @return max Index Angle
     */
    double getMaxIndexAngle();
}
