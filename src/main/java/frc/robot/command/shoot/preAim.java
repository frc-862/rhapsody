package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class preAim extends Command {

    private Flywheel flywheel;
    private Pivot pivot;
    private Swerve drivetrain;

    public preAim(Flywheel flywheel, Pivot pivot, Swerve drivetrain) {
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.drivetrain = drivetrain;

        addRequirements(flywheel, pivot);
    }

    @Override
    public void initialize() {
        System.out.println("AUTO - PRE AIM START");
    }

    @Override
    public void execute() {
        double distance = drivetrain.distanceToSpeaker();
        pivot.setTargetAngle(calculateTargetAngle(distance));
        flywheel.setAllMotorsRPM(calculateTargetRPM(distance));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AUTO - PRE AIM END");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Calculate Pivot Target angle (in degrees)
     *
     * @param distance from the speaker
     * @return Angle to set pivot to
     */
    public double calculateTargetAngle(double distance) {
        if (Constants.isMercury()) {
            return ShooterConstants.TUBE_ANGLE_MAP.get(distance);
        }
        return ShooterConstants.STEALTH_ANGLE_MAP.get(distance);
    }

    /**
     * Calculate Flywheel Target RPM (in RPM)
     *
     * @param distance from the speaker
     * @return RPM to set the Flywheels
     */
    public double calculateTargetRPM(double distance) {
        if (Constants.isMercury()) {
            return ShooterConstants.TUBE_SPEED_MAP.get(distance);
        }
        return ShooterConstants.STEALTH_SPEED_MAP.get(distance);
    }
}
