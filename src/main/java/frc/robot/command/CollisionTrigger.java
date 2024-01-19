package frc.robot.command;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.VisionConstants;

public class CollisionTrigger extends Trigger {
    // states Collison subsystem
    Swerve drivetrain;
    public CollisionTrigger(BooleanSupplier condition, Swerve drivetrain) {
        // initializes varaibles from Trigger and Collision classes
        super(condition);
        this.drivetrain = drivetrain;
    }
    @Override
    // our boolean for our custom trigger
    public boolean getAsBoolean() {
        // checks if the absolute value of our pitch or roll is greater than our deadzone
        return Math.abs(drivetrain.getPigeon2().getPitch().getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE || Math.abs(drivetrain.getPigeon2().getRoll().getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE;
    }
    
}
