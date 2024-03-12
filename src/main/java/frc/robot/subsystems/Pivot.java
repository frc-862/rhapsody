package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Pivot extends Subsystem{

	/**
     * Sets the target angle of the pivot
     * @param angle Angle of the pivot
     */
    public void setTargetAngle(double angle);

    public void setPower(double power);

    /**
     * @return The current angle of the pivot in degrees
     */
    public double getAngle();

    /**
     * @return Whether or not the pivot is on target, within Angle tolerance
     */
    public boolean onTarget();

    /**
     * Gets forward limit switch
     * @return true if pressed
     */
    public boolean getForwardLimit();

    /**
     * Gets reverse limit switch
     * @return true if pressed
     */
    public boolean getReverseLimit();

    /**
     * @return The bias to add to the target angle of the pivot
     */
    public double getBias();

    /**
     * Increases the bias of the pivot by set amount
     */
    public void increaseBias();

    /**
     * Decreases the bias of the pivot by set amount
     */
    public void decreaseBias();

    /**
     * Resets the bias of the pivot
     */
    public void resetBias();

    /**
     * @param angle angle to set the pivot angle to
     */
    public void resetAngle(double angle);

    /**
     * @return current stow angle
     */
    public double getStowAnlge();

    /**
     * @return max Index Angle
     */
    public double getMaxIndexAngle();
}
