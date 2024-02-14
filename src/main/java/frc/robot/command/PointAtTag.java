package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

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

	private FieldCentric slow;
	private FieldCentric drive;
	
	private int limelightPrevPipeline = 0;
	private double pidOutput;
	private double targetHeading;
	private Translation2d targetPose;

	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param targetX the x coordinate of the target
	 * @param targetY the y coordinate of the target
	 * @param drivetrain to request movement 
	 * @param limelights to get the limelight from
	 * @param driver the driver's controller, used for drive input
	 */
	public PointAtTag(int targetX, int targetY, Swerve drivetrain, Limelights limelights, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		
		//TODO Figure out which of these is the right one to use 
		limelight = limelights.getStopMe();

		limelightPrevPipeline = limelight.getPipeline();

		limelight.setPipeline(VisionConstants.TAG_PIPELINE);
		
		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
		slow = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

		targetPose = new Translation2d(targetX, targetY);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		// targetPose = limelight.getCamPoseTargetSpace().getTranslation().toTranslation2d();
		headingController.enableContinuousInput(-180, 180);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		headingController.setTolerance(LightningShuffleboard.getDouble("PointAtTag", "Tolarance", 4));
		headingController.setP(LightningShuffleboard.getDouble("PointAtTag", "P", 0.1));
		headingController.setI(LightningShuffleboard.getDouble("PointAtTag", "I", 0));
		headingController.setD(LightningShuffleboard.getDouble("PointAtTag", "D", 1));


		Pose2d pose = drivetrain.getPose().get();
		var deltaX = targetPose.getX() - pose.getX();
		var deltaY = targetPose.getY() - pose.getY();

		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
		// targetHeading = Math.toDegrees(targetPose.getRotation().getAngle());
		pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);



		// targetHeading = limelight.getTargetX();
		// pidOutput = headingController.calculate(targetHeading, 0);
		LightningShuffleboard.setDouble("PointAtTag", "Delta Y", deltaY);
		LightningShuffleboard.setDouble("PointAtTag", "Delta X", deltaX);
		LightningShuffleboard.setDouble("PointAtTag", "Target Heading2", targetHeading);
		LightningShuffleboard.setDouble("PointAtTag", "Target Pose Y", targetPose.getY());
		LightningShuffleboard.setDouble("PointAtTag", "Target Pose X", targetPose.getX());
		LightningShuffleboard.setDouble("PointAtTag", "Current Pose Y", pose.getY());
		LightningShuffleboard.setDouble("PointAtTag", "Current PoseX", pose.getX());
		LightningShuffleboard.setDouble("PointAtTag", "Drivetrain Angle", pose.getRotation().getDegrees());
		LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);

		// drivetrain.applyRequest(() -> drive
		// 				.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
		// 						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
		// 				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
		// 						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive left with negative X (left)
		// 				.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
		// 						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
		// 						* DrivetrainConstants.ROT_MULT) // Drive counterclockwise with negative X (left)
		// 		);
		if (driver.getRightBumper()) {
			drivetrain.setControl(slow.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
				.withRotationalRate(pidOutput) // Drive counterclockwise with negative X (left)
			);
		} else {
			drivetrain.setControl(drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive left with negative X (left)
				.withRotationalRate(pidOutput) // Rotate toward the desired direction
			); // Drive counterclockwise with negative X (left)
		}
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