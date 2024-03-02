package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PointAtPoint extends Command {

	private Swerve drivetrain;
	private XboxController driver;

	private double pidOutput;
	private double targetHeading;
	private Translation2d targetPose;

	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param targetX the x coordinate of the target
	 * @param targetY the y coordinate of the target
	 * @param drivetrain to request movement
	 * @param driver the driver's controller, used for drive input
	 */
	public PointAtPoint(double targetX, double targetY, Swerve drivetrain, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		this.targetPose = new Translation2d(targetX, targetY);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		headingController.enableContinuousInput(-180, 180);
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Heading", () -> targetHeading);
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Pose Y", () -> targetPose.getY());
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Pose X", () -> targetPose.getX());
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Pid Output", () -> pidOutput);
	}

	@Override
	public void execute() {
		Pose2d pose = drivetrain.getPose().get();
		var deltaX = targetPose.getX() - pose.getX();
		var deltaY = targetPose.getY() - pose.getY();

		// LightningShuffleboard.setDouble("PointAtTag", "Delta Y", deltaY);
		// LightningShuffleboard.setDouble("PointAtTag", "Delta X", deltaX);

		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
		targetHeading += 180;
		pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);

		drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), pidOutput);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}