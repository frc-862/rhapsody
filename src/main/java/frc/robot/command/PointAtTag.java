package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
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

		//TODO Figure out which of these is the right one to use 
		limelight = limelights.getStopMe();

		limelightPrevPipeline = limelight.getPipeline();

		limelight.setPipeline(VisionConstants.TAG_PIPELINE);

	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		headingController.enableContinuousInput(-180, 180);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		headingController.setTolerance(LightningShuffleboard.getDouble("PointAtTag", "Tolarance", 4));
		headingController.setP(LightningShuffleboard.getDouble("PointAtTag", "P", 0.1));
		headingController.setI(LightningShuffleboard.getDouble("PointAtTag", "I", 0));
		headingController.setD(LightningShuffleboard.getDouble("PointAtTag", "D", 1));

		targetHeading = limelight.getTargetX();
		pidOutput = headingController.calculate(targetHeading, 0);
		
		LightningShuffleboard.setDouble("PointAtTag", "Target Heading2", targetHeading);
		LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);

		// TODO test drives and test the deadbands
		drivetrain.applyRequestField(() -> -driver.getLeftY(), () -> -driver.getLeftX(), pidOutput, ControllerConstants.DEADBAND);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightPrevPipeline);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}