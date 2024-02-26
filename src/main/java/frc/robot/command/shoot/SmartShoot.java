package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CandConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.Constants.ShooterConstants.ShootingState;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SmartShoot extends Command {

	final Flywheel flywheel;
	final Pivot pivot;
	final Swerve drivetrain;
	final Indexer indexer;
	final LEDs leds;

	private ShootingState state = ShootingState.AIM;
	private double shotTime = 0d;

	/**
	 * SmartShoot to control flywheel, pivot, drivetrain, and indexer
	 * @param flywheel subsystem to set target RPM
	 * @param pivot subsystem to set target angle
	 * @param drivetrain subsystem to get distance from speaker
	 * @param indexer subsystem to set power
	 * @param leds subsystem to provide driver feedback
	 */
	public SmartShoot(Flywheel flywheel, Pivot pivot, Swerve drivetrain, Indexer indexer, LEDs leds) {
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.drivetrain = drivetrain;
		this.indexer = indexer;
		this.leds = leds;

		addRequirements(pivot, flywheel, indexer);
	}

	@Override
	public void initialize() {
		//Always start with aiming
		state = ShootingState.AIM;
	}

	@Override
	public void execute() {
		// Distance from current pose to speaker pose
		double distance = drivetrain.distanceToSpeaker();

		// Logging
		LightningShuffleboard.setDouble("Smart-Shoot", "Distance", distance);
		LightningShuffleboard.setString("Smart-Shoot", "State", state.toString());

		switch(state) {
			case AIM:
				// Default state remains here until pivot + Flywheel are on target
				pivot.setTargetAngle(calculateTargetAngle(distance));
				flywheel.setAllMotorsRPM(calculateTargetRPM(distance));
				if (onTarget()) { // Moves to next state and logs time
					state = ShootingState.SHOOT;
					shotTime = Timer.getFPGATimestamp();
				}
			break;
			case SHOOT:
				// Continues aiming, indexs to shoot
				pivot.setTargetAngle(calculateTargetAngle(distance));
				flywheel.setAllMotorsRPM(calculateTargetRPM(distance));
				indexer.indexUp();
				// Once shoot critera met moves to shot
				if(Timer.getFPGATimestamp() - shotTime >= CandConstants.TIME_TO_SHOOT){
					state = ShootingState.SHOT;
				}
			break;
			case SHOT:
				// Provides driver feed back and ends command
				new TimedCommand(RobotContainer.hapticDriverCommand(), 1d).schedule();
				leds.enableState(LED_STATES.SHOT).withTimeout(2).schedule();
			break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(PivotConstants.STOW_ANGLE);
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return state == ShootingState.SHOT;
	}

	/**
	 * Checks if Flywheel and Pivot are in range of target angle
	 * @return boolean on Target
	 */
	public boolean onTarget() {
		return flywheel.allMotorsOnTarget() && pivot.onTarget();
	}

	/**
	 * Calculate Pivot Target angle (in degrees)
	 * @param distance from the speaker
	 * @return Angle to set pivot to
	 */
	public double calculateTargetAngle(double distance) {
		return ShooterConstants.ANGLE_MAP.get(distance);
	}

	/**
	 * Calculate Flywheel Target RPM (in RPM)
	 * @param distance from the speaker
	 * @return RPM to set the Flywheels
	 */
	public double calculateTargetRPM(double distance) {
		return ShooterConstants.SPEED_MAP.get(distance);
	}
}