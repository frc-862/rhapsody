package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrAinConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class PointAtTag extends Command {

	private Swerve drivetrain;
	private Limelight limelight;
	private int limelightId = 0;
	private double targetHeading;
	private double lockedOnHeading;
	private double pidOutput;
	private SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	private XboxController driver;
	private FieldCentric drive;
	private FieldCentric slow;
	private boolean useLimelights;
	

	PIDController headingController = VisionConstants.HEADING_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 * @param driver the driver's controller, used for drive input
	 * @param limelight_name the name of the limelight to use
	 * @param useLimelights to get if we want to use vision data or not
	 * @param noteDetection to get if we want to use this for note detection or april tag detection
	 */
	public PointAtTag(Swerve drivetrain, XboxController driver, String limelight_name, boolean useLimelights, boolean noteDetection) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		this.useLimelights = useLimelights;


		//TODO Figure out which of these is the right one to use 
		// for (var l : drivetrain.getLimelights()) { 
		// 	if (l.getName().equals(limelight_name)) {
		// 		limelight = l;
		// 	}
		// }

		limelight = drivetrain.getLimelights()[0];

		limelightId = limelight.getPipeline();
		if (noteDetection){
			limelight.setPipeline(VisionConstants.NOTE_PIPELINE);
		} else {
			limelight.setPipeline(VisionConstants.TAG_PIPELINE);
		}

		
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
		if (useLimelights) {

			targetHeading = limelight.getTargetX();

			SwerveRequest.RobotCentric pointAtTag = new SwerveRequest.RobotCentric();
			pidOutput = headingController.calculate(0, targetHeading);
			drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));

		} else {

			lockedOnHeading = LightningShuffleboard.getDouble("PointAtTag", "LockOnHeading", 0);
			LightningShuffleboard.setDouble("PointAtTag", "Drivetrain Angle", drivetrain.getPigeon2().getAngle());
			targetHeading = lockedOnHeading - drivetrain.getPigeon2().getAngle();


			SwerveRequest.FieldCentric pointAtTag = new SwerveRequest.FieldCentric();
			pidOutput = headingController.calculate(0, targetHeading);
			drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));

		}

		LightningShuffleboard.setDouble("PointAtTag", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);

		if (driver.getRightBumper()) {
			drivetrain.setControl(slow.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
				.withRotationalRate(-pidOutput) // Drive counterclockwise with negative X (left)
			);
		} else {
			drivetrain.setControl(drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
		  .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive left with negative X (left)
		  .withRotationalRate(-pidOutput) // Rotate toward the desired direction
		  ); // Drive counterclockwise with negative X (left)
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightId);
		drivetrain.applyRequest(() -> brake); // TODO test if this applies brake
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
