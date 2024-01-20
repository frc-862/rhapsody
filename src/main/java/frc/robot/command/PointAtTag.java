package frc.robot.command;

import java.util.Arrays;

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
	private double pidOutput;
	private SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	PIDController headingController = new PIDController(0.1, 0, 0); // TODO make constant and tune
	private XboxController driver;
	private FieldCentric drive;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 */
	public PointAtTag(Swerve drivetrain, XboxController driver, String limelight_name) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		for (var l : drivetrain.getLimelights()) {
			if (l.getName().equals(limelight_name)) {
				limelight = l;
			}
		}
		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		limelightId = limelight.getPipeline();
		limelight.setPipeline(2);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		for (Limelight limelight : Limelight.filterLimelights(limelights)) {
			targetHeading = limelight.getTargetX();
		}

		LightningShuffleboard.setDouble("PointAtTag", "Target Heading", targetHeading);

		headingController.setP(LightningShuffleboard.getDouble("PointAtTag", "P", 0.1));
		headingController.setD(LightningShuffleboard.getDouble("PointAtTag", "D", 0));

		pidOutput = headingController.calculate(0, targetHeading);
		LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);
		SwerveRequest.RobotCentric pointAtTag = new SwerveRequest.RobotCentric();
		drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));
		// drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
		//   .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive left with negative X (left)
		//   .withRotationalRate(-pidOutput)); // Drive counterclockwise with negative X (left)

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
