package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class PointAtTag extends Command {

	private Swerve drivetrain;
	private Limelight limelight;
	private XboxController driver;
	
	private int limelightPrevPipeline = 0;
	private double pidOutput;
	private double targetHeading;

	private Translation2d targetPose;
	private double deltaX;
	private double deltaY;


	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 * @param limelights to get the limelight from
	 * @param driver the driver's controller, used for drive input
	 */
	public PointAtTag(Swerve drivetrain, Limelights limelights, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;

		// TODO Figure out which of these is the right one to use
		limelight = limelights.getStopMe();

		limelightPrevPipeline = limelight.getPipeline();

		limelight.setPipeline(VisionConstants.SPEAKER_PIPELINE);

		targetPose = new Translation2d(0, 5.43);
	}

	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);

		headingController.enableContinuousInput(-180, 180);

		initLogging();
	}

	private void initLogging() {
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Delta Y", () -> deltaY);
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Delta X", () -> deltaX);
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Heading", () -> targetHeading);
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Pose Y", () -> targetPose.getY());
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Target Pose X", () -> targetPose.getX());
		LightningShuffleboard.setDoubleSupplier("PointAtTag", "Pid Output", () -> pidOutput);
	}

	@Override
	public void execute() {

		Pose2d pose = drivetrain.getPose().get();
		deltaX = targetPose.getX() - pose.getX();
		deltaY = targetPose.getY() - pose.getY();

		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
		// targetHeading = Math.toDegrees(targetPose.getRotation().getAngle());
		pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);

		// targetHeading = limelight.getTargetX();
		// pidOutput = headingController.calculate(targetHeading, 0);

		drivetrain.setFieldDriver(-driver.getLeftY(), -driver.getLeftX(), -pidOutput);
	}

	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightPrevPipeline);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}