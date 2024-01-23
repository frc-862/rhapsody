package frc.robot.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	
	Shooter shooter;
	Indexer indexer;
	Swerve drivetrain;

	double distancetoTarget; // FROM LIMELIGHT or form OTF
	private BooleanSupplier copilotAButton;

	/**
	 * Creates a new Shoot.
	 * @param Shooter 
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
