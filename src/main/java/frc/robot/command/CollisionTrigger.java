package frc.robot.command;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.event.EventLoop;
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
        // checks if our magnitude is greater than our deadzone
        return collision.getMagnitude() > VisionConstants.COLLISION_DEADZONE;
    }
    
}
