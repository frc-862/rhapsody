package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PointAtPoint extends Command {

	private static final double MIN_POWER = 0.3;
	private Swerve drivetrain;
	private XboxController driver;

	private double pidOutput;
	private double targetHeading;
	private Translation2d targetPose;
	private Translation2d originalTargetPose;

	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * 
	 * @param targetX    the x coordinate of the target
	 * @param targetY    the y coordinate of the target
	 * @param drivetrain to request movement
	 * @param driver     the driver's controller, used for drive input
	 */
	public PointAtPoint(double targetX, double targetY, Swerve drivetrain, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		this.originalTargetPose = new Translation2d(targetX, targetY);

		addRequirements(drivetrain);
	}

	public PointAtPoint(Translation2d targetPose, Swerve drivetrain, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		this.originalTargetPose = targetPose;

		addRequirements(drivetrain);
	}

	private boolean isBlueAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get() == Alliance.Blue;
	}

	private Translation2d swapAlliance(Translation2d pose) {
		return new Translation2d(VisionConstants.FIELD_LIMIT.getX() - pose.getX(), pose.getY());
	}

	private boolean inTolerance() {
		return Math.abs(targetHeading - drivetrain.getPose().getRotation().getDegrees()) < DrivetrainConstants.ALIGNMENT_TOLERANCE;
	}

	@Override
	public void initialize() {
		headingController.enableContinuousInput(0, 360);
		if (isBlueAlliance()) {
			targetPose = originalTargetPose;
		} else {
			targetPose = swapAlliance(originalTargetPose);
		}
	}

	@Override
	public void execute() {
		Pose2d pose = drivetrain.getPose();
		var deltaX = targetPose.getX() - pose.getX();
		var deltaY = targetPose.getY() - pose.getY();

		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 360 + 180;
		targetHeading %= 360;
		pidOutput = headingController.calculate(pose.getRotation().getDegrees() % 360, targetHeading);

		LightningShuffleboard.setDouble("PointAtPoint", "Delta Y", deltaY);
		LightningShuffleboard.setDouble("PointAtPoint", "Delta X", deltaX);
		LightningShuffleboard.setDouble("PointAtPoint", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("PointAtPoint", "Target Pose Y", targetPose.getY());
		LightningShuffleboard.setDouble("PointAtPoint", "Target Pose X", targetPose.getX());
		LightningShuffleboard.setDouble("PointAtPoint", "Pid Output", pidOutput);
		LightningShuffleboard.setDouble("PointAtPoint", "Current", pose.getRotation().getDegrees());

		if (!inTolerance() && Math.abs(pidOutput) < MIN_POWER) {
			pidOutput = Math.signum(pidOutput) * MIN_POWER;
		}

		drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), pidOutput);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		// if(DriverStation.isAutonomous()) {
			return inTolerance();
		// }
		// return false;
	}
}