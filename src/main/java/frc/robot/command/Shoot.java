package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.ShooterTargeting;

public class Shoot extends Command {
    Flywheel flywheel;
    Pivot pivot;
    ShooterTargeting shooterTargeting;

    public Shoot(Flywheel flywheel, Pivot pivot, ShooterTargeting shooterTargeting) {
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.shooterTargeting = shooterTargeting;

        addRequirements(flywheel, pivot, shooterTargeting);
        }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        flywheel.setRPM(shooterTargeting.getTargetFlywheelRPM());
        pivot.setAngle(shooterTargeting.getTargetFlywheelAngle());
    }

    @Override
    public void execute() {
        if(shooterTargeting.readyToFire());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheel.setRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
