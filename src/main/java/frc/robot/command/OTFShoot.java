package frc.robot.command;

import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class OTFShoot extends Command {

	private Swerve drivetrain;
	private XboxController driver;

	private FieldCentric slow;
	private FieldCentric drive;
	
	private double pidOutput;

	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 * @param driver the driver's controller, used for drive input
	 */
	public OTFShoot(Swerve drivetrain, XboxController driver) {
		this.drivetrain = drivetrain;
		this.driver = driver;
		
		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
		slow = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	}
	
	@Override
	public void initialize() {

		headingController.enableContinuousInput(-180, 180);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Pose2d pose = drivetrain.getPose().get();
		double deltaX = VisionConstants.SPEAKER_LOCATION.getX() - pose.getX();
		double deltaY = VisionConstants.SPEAKER_LOCATION.getY() - pose.getY();
    double deltaZ = VisionConstants.SPEAKER_LOCATION.getZ() - 0.75;
    
    // Getting robot values 
    double xAccleration = drivetrain.getPigeon2().getAccelerationX().getValue();
    double yAccleration = drivetrain.getPigeon2().getAccelerationY().getValue();

    double xVelocity = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
    double yVelocity = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;

    double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    double timeToTarget = distanceToTarget / 31.2927;
    
    // Change of final peice velocity due to final robot acceleration
    double xVelocityFinal = xVelocity + xAccleration * timeToTarget;
    double yVelocityFinal = yVelocity + yAccleration * timeToTarget;

    double deltaXFinalVelocity = xVelocityFinal * timeToTarget;
    double deltaYFinalVelocity = yVelocityFinal * timeToTarget;

    // Change of target pose due to acceleration and velocity
    double targetX = VisionConstants.SPEAKER_LOCATION.getX() + deltaXFinalVelocity;
    double targetY = VisionConstants.SPEAKER_LOCATION.getY() + deltaYFinalVelocity;

    //Change of final Robot pose due to acceleration and velocity
    double finalX = pose.getX() + deltaXFinalVelocity + 0.5 * xAccleration * timeToTarget * timeToTarget;
    double finalY = pose.getY() + deltaYFinalVelocity + 0.5 * yAccleration * timeToTarget * timeToTarget;

    // Get final Delta X and Delta Y
    double finalDeltaX = targetX - finalX;
    double finalDeltaY = targetY - finalY;

    //getting the angle to the target
    double targetHeading = Math.toDegrees(Math.atan2(finalDeltaX, finalDeltaY));
    
    pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);

    LightningShuffleboard.setDouble("OTF Shooting", "Distance to Speaker", distanceToTarget);
		LightningShuffleboard.setDouble("OTF Shooting", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("OTF Shooting", "Delta X", deltaX);
		LightningShuffleboard.setDouble("OTF Shooting", "Delta Y", deltaY);
		LightningShuffleboard.setDouble("OTF Shooting", "Delta Z", deltaZ);
		LightningShuffleboard.setDouble("OTF Shooting", "X Velocity", xVelocity);
		LightningShuffleboard.setDouble("OTF Shooting", "Y Velocity", yVelocity);
    LightningShuffleboard.setDouble("OTF Shooting", "X Accleration", xAccleration);
    LightningShuffleboard.setDouble("OTF Shooting", "Y Accleration", yAccleration);
    LightningShuffleboard.setDouble("OTF Shooting", "Time to Target", timeToTarget);
    LightningShuffleboard.setDouble("OTF Shooting", "Final X", finalX);
    LightningShuffleboard.setDouble("OTF Shooting", "Final Y", finalY);
    LightningShuffleboard.setDouble("OTF Shooting", "Final Delta X", finalDeltaX);
    LightningShuffleboard.setDouble("OTF Shooting", "Final Delta Y", finalDeltaY);
    LightningShuffleboard.setDouble("OTF Shooting", "Target X", targetX);
    LightningShuffleboard.setDouble("OTF Shooting", "Target Y", targetY);
    LightningShuffleboard.setDouble("OTF Shooting", "X Velocity Final", xVelocityFinal);
    LightningShuffleboard.setDouble("OTF Shooting", "Y Velocity Final", yVelocityFinal);
    LightningShuffleboard.setDouble("OTF Shooting", "Delta X Final Velocity", deltaXFinalVelocity);
    LightningShuffleboard.setDouble("OTF Shooting", "Delta Y Final Velocity", deltaYFinalVelocity);
    LightningShuffleboard.setDouble("OTF Shooting", "PID Output", pidOutput);
    LightningShuffleboard.setDouble("OTF Shooting", "Robot X", pose.getX());
    LightningShuffleboard.setDouble("OTF Shooting", "Robot Y", pose.getY());
    LightningShuffleboard.setDouble("OTF Shooting", "Robot Heading", pose.getRotation().getDegrees());


    

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

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}