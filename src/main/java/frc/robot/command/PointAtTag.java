package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
		
		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
		slow = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double targetPoseX = 0;
		double targetPoseY = 3;
		
		Pose2d pose = drivetrain.getPose().get();
		targetHeading = Math.toDegrees(Math.atan2( targetPoseY - drivetrain.getPose().get().getY(), targetPoseX - drivetrain.getPose().get().getX()));
		pidOutput = headingController.calculate((pose.getRotation().getDegrees() % 360.0), targetHeading % 360.0);


		// targetHeading = limelight.getTargetX();
		// pidOutput = headingController.calculate(targetHeading, 0);

		LightningShuffleboard.setDouble("PointAtTag", "Drivetrain Angle", (pose.getRotation().getDegrees() % 360.0));
		LightningShuffleboard.setDouble("PointAtTag", "Target Heading", targetHeading);
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