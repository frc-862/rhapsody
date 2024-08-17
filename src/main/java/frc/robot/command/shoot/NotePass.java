package frc.robot.command.shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PassConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class NotePass extends Command {

	private final Swerve drivetrain;
	private final Flywheel flywheel;
	private final Pivot pivot;

	private Translation2d targetPose;
	private double currentHeading;
	private double targetHeading;
	private double distanceToSpeaker;
	private double feedForwardOutput;
	private double pidOutput;
	private PIDController pidController = PassConstants.PASS_CONTROLLER;
	private SimpleMotorFeedforward feedforward = PassConstants.FEED_FORWARD;
	private XboxControllerFilter driver;

	private DoubleLogEntry currentHeadingLog;
	private DoubleLogEntry targetHeadingLog;
	private DoubleLogEntry pidOutputLog;
	private DoubleLogEntry distanceToSpeakerLog;
	private BooleanLogEntry headingOnTargetLog;
	private BooleanLogEntry shooterOnTargetLog;

	/**
	 * Creates a new NotePass.
	 *
	 * @param drivetrain subsystem
	 * @param pivot      subsystem
	 * @param flywheel   subsystem
	 * @param driver     the driver's controller, used for drive input
	 */
	public NotePass(Swerve drivetrain, Flywheel flywheel, Pivot pivot, XboxControllerFilter driver) {
		this.drivetrain = drivetrain;
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.driver = driver;

		addRequirements(flywheel, pivot, drivetrain);
	}

	@Override
	public void initialize() {
		/*
		 * Because the flywheels are on one side, the note will spin slightly
		 * which is why we want to use the corner pose on blue and
		 * the speaker pose on red.
		 * I may be going crazy but whatever!!
		 */
		if (isBlueAlliance()) {
			targetPose = DrivetrainConstants.BLUE_SPEAKER_POSE;
		} else {
			targetPose = DrivetrainConstants.RED_CORNER_POSE;
		}

		initLogging();
	}

	@Override
	public void execute() {
		Pose2d pose = drivetrain.getPose();
		var deltaX = targetPose.getX() - pose.getX();
		var deltaY = targetPose.getY() - pose.getY();

		distanceToSpeaker = drivetrain.distanceToSpeaker();
		currentHeading = (pose.getRotation().getDegrees() + 360) % 360;

		// Calculate vector to target, add 180 to make it point backwards
		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180;
		targetHeading = (targetHeading + 360) % 360; // Modulo 360 to keep it in the range of 0-360

		pidOutput = pidController.calculate(currentHeading, targetHeading);
		feedForwardOutput = feedforward.calculate(pidOutput);

		if (inTolerance()) {
			feedForwardOutput = 0;
		}

		drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), feedForwardOutput);

		flywheel.setAllMotorsRPM(ShooterConstants.NOTEPASS_SPEED_MAP.get(distanceToSpeaker) + flywheel.getBias());
		pivot.setTargetAngle(ShooterConstants.NOTEPASS_ANGLE_MAP.get(distanceToSpeaker) + pivot.getBias());

		if (flywheel.allMotorsOnTarget() && pivot.onTarget() && inTolerance()) {
			new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
			new TimedCommand(RobotContainer.hapticDriverCommand(), 1d).schedule();
		}

		updateLogging();
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	private boolean isBlueAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get() == Alliance.Blue;
	}

	private boolean inTolerance() {
		double difference = Math.abs(currentHeading - targetHeading);
		difference = difference > 180 ? 360 - difference : difference;
		return difference <= PassConstants.POINT_TOLERANCE;
	}

	/**
	 * initialize logging
	 */
	private void initLogging() {
		DataLog log = DataLogManager.getLog();

		currentHeadingLog = new DoubleLogEntry(log, "/NotePass/CurrentHeading");
		targetHeadingLog = new DoubleLogEntry(log, "/NotePass/TargetHeading");
		pidOutputLog = new DoubleLogEntry(log, "/NotePass/PIDoutput");
		distanceToSpeakerLog = new DoubleLogEntry(log, "/NotePass/DistanceToSpeaker");

		headingOnTargetLog = new BooleanLogEntry(log, "/NotePass/Heading-OnTarget");
		shooterOnTargetLog = new BooleanLogEntry(log, "/NotePass/Shooter-OnTarget");

		if (!DriverStation.isFMSAttached()) {
			LightningShuffleboard.setBoolSupplier("Note-Pass", "In tolerance", () -> inTolerance());
			LightningShuffleboard.setDoubleSupplier("Note-Pass", "CurrentHeading", () -> currentHeading);
			LightningShuffleboard.setDoubleSupplier("Note-Pass", "TargetHeading", () -> targetHeading);
			LightningShuffleboard.setDoubleSupplier("Note-Pass", "Distance to Speaker", () -> distanceToSpeaker);
		}
	}

	/**
	 * update logging
	 */
	public void updateLogging() {
		currentHeadingLog.append(currentHeading);
		targetHeadingLog.append(targetHeading);
		pidOutputLog.append(feedForwardOutput);
		distanceToSpeakerLog.append(distanceToSpeaker);

		headingOnTargetLog.append(inTolerance());
		shooterOnTargetLog.append(flywheel.allMotorsOnTarget() && pivot.onTarget());
	}
}
