package frc.robot.command;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Collision;
import frc.robot.Constants.VisionConstants;

public class CollisionTrigger extends Trigger {
    // states Collison subsystem
    Collision collision;
    public CollisionTrigger(BooleanSupplier condition, Collision collision) {
        // initializes varaibles from Trigger and Collision classes
        super(condition);
        this.collision = collision;
    }
    @Override
    // our boolean for our custom trigger
    public boolean getAsBoolean() {
        // checks if the absolute value of our pitch or roll is greater than our deadzone
        return Math.abs(collision.getPitch()) > VisionConstants.COLLISION_DEADZONE || Math.abs(collision.getRoll()) > VisionConstants.COLLISION_DEADZONE;
    }
    
}
