package frc.robot.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Swerve drivetrain;
	private BooleanSupplier copilotAButton;

	double distancetoTarget; // FROM LIMELIGHT or form OTF

	/**
	 * Creates Shoot command
	 * @param shooter Shooter logic, calls Flywheel and Pivot
	 * @param indexer 
	 * @param drivetrain for pose
	 * @param copilotAButton To trigger the shot
	 */
	public Shoot(Shooter shooter, Indexer indexer, Swerve drivetrain, BooleanSupplier copilotAButton) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.drivetrain = drivetrain;
		this.copilotAButton = copilotAButton;

		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		distancetoTarget = shooter.getDistanceToTarget();
		shooter.setState(SHOOTER_STATES.STOW);
	}

	@Override
	public void execute() { // DUMB but good enough for now 
		distancetoTarget = shooter.getDistanceToTarget();
		if (drivetrain.inWing()) {
            if (copilotAButton.getAsBoolean()) { 
                shooter.setState(SHOOTER_STATES.SHOOT);
            } else {
                shooter.setState(SHOOTER_STATES.PRIME);
            }
        } else {
            shooter.setState(SHOOTER_STATES.STOW);
        }
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setState(SHOOTER_STATES.STOW);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}