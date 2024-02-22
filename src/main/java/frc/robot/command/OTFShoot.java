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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class OTFShoot extends Command {

	private Swerve drivetrain;
	private XboxController driver;
	private Pivot pivot;
	private Flywheel flywheel;
	private Indexer indexer;

	private double fireTime = Timer.getFPGATimestamp() + ShooterConstants.OTF_READY_TIME;

	private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement
	 * @param driver the driver's controller, used for drive input
	 * @param pivot the pivot subsystem
	 * @param flywheel the flywheel subsystem
	 * @param indexer the indexer subsystem
	 */
	public OTFShoot(Swerve drivetrain, XboxController driver, Pivot pivot, Flywheel flywheel, Indexer indexer) {

		this.drivetrain = drivetrain;
		this.driver = driver;
		this.pivot = pivot;
		this.flywheel = flywheel;
		this.indexer = indexer;

	}

	@Override
	public void initialize() {

		headingController.enableContinuousInput(-180, 180);
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);

		addRequirements(drivetrain, pivot, flywheel, indexer);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Time until the shot
		double timeUntilFire = fireTime - Timer.getFPGATimestamp();
		// Robot pose
		Pose2d pose = drivetrain.getPose().get();

		// Robot Acceleration
		double accelerationScaler = 1;
		double robotAccelerationX = drivetrain.getPigeon2().getAccelerationX().getValue();
		double robotAccelerationY = drivetrain.getPigeon2().getAccelerationY().getValue();

		// Robot velocity
		//TODO: get real value
		double velocityDeadback = 0.2;
		double veloctiyScaler = 1;
		double robotVelocityX = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
		double robotVelocityY = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
		
		// If the robot is not moving, don't compensate for velocity
		if ((Math.abs(robotVelocityX) < velocityDeadback && Math.abs(robotVelocityY) < velocityDeadback) || (LightningShuffleboard.getBool("OTF Shooting", "Velocity Compensating", false))){
			veloctiyScaler = 0;
		}

		// Acceleration don't compensate for accleration
		if (LightningShuffleboard.getBool("OTF Shooting", "Acceleration Compensating", false)) {
			accelerationScaler = 0;
		}

		// Speed of the shot
		//TODO: get real value
		double shotSpeed = 0.5;

		// Get the distance and time to the speaker
		double distanceToSpeaker = Math.sqrt(Math.pow(
		VisionConstants.SPEAKER_LOCATION.getX() - pose.getX(), 2)
		+ Math.pow(VisionConstants.SPEAKER_LOCATION.getY() - pose.getY(), 2)
		+ Math.pow(VisionConstants.SPEAKER_LOCATION.getZ() - 0.75, 2));
		double timeToSpeaker = distanceToSpeaker / shotSpeed;

		// Velocity of the robot at release point (Vf = Vi + AT)
		double robotReleaseVelocityX = robotVelocityX * veloctiyScaler + (robotAccelerationX * accelerationScaler * timeUntilFire);
		double robotReleaseVelocityY = robotVelocityY * veloctiyScaler + (robotAccelerationY * accelerationScaler * timeUntilFire);

		// Offset of landing peice location bassed on robot velocity at release point (Delta = VT)
		double pieceDeltaX = robotReleaseVelocityX * timeToSpeaker;
		double pieceDeltaY = robotReleaseVelocityY * timeToSpeaker;

		// Change of target pose landing peice location bassed on robot velocity at release point
		double targetX = VisionConstants.SPEAKER_LOCATION.getX() + pieceDeltaX;
		double targetY = VisionConstants.SPEAKER_LOCATION.getY() + pieceDeltaY;

		//Change of final Robot pose due to acceleration and velocity (X = Xi + ViT + 0.5AT^2)
		double releaseRobotPoseX = pose.getX() + (robotVelocityX * timeUntilFire  * veloctiyScaler) + (0.5 * robotAccelerationX * timeUntilFire * timeUntilFire * accelerationScaler);
		double releaseRobotPoseY = pose.getY() + (robotVelocityY * timeUntilFire  * veloctiyScaler) + (0.5 * robotAccelerationY * timeUntilFire * timeUntilFire * accelerationScaler);

		// Get final Delta X and Delta Y to find the target heading of the robot
		double headingDeltaX = targetX - releaseRobotPoseX;
		double headingDeltaY = targetY - releaseRobotPoseY;

		//getting the angle to the target (Angle = arctan(Dy, Dx))
		double targetHeading = Math.toDegrees(Math.atan2(headingDeltaY, headingDeltaX));

		// Heading of Robot without math and its delta
		double basicDeltaX = VisionConstants.SPEAKER_LOCATION.getX() - pose.getX();
		double basicDeltaY = VisionConstants.SPEAKER_LOCATION.getY() - pose.getY();
		double basicHeading = Math.toDegrees(Math.atan2(basicDeltaY, basicDeltaX));
		double basicDelta = Math.abs(targetHeading - basicHeading);

		double pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);

		double velocityToTarget = new Translation2d(robotReleaseVelocityX, robotReleaseVelocityY).rotateBy(new Rotation2d(targetHeading - pose.getRotation().getDegrees())).getX();
		double deltaPieceTarget = velocityToTarget * timeToSpeaker;
		double pivotTargetX = VisionConstants.SPEAKER_LOCATION.getX() - deltaPieceTarget;
		double pivotTargetY = VisionConstants.SPEAKER_LOCATION.getY();
		double pivotTargetDeltaX = pivotTargetX - pose.getX();
		double pivotTargetDeltaY = pivotTargetY - pose.getY();
		double pivotTargetAngle = Math.toDegrees(Math.atan2(pivotTargetDeltaY, pivotTargetDeltaX));

		LightningShuffleboard.setDouble("OTF Shooting", "Distance to Speaker", distanceToSpeaker);
		LightningShuffleboard.setDouble("OTF Shooting", "Rotated Target Heading", (targetHeading + 360) % 360);
		LightningShuffleboard.setDouble("OTF Shooting", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("OTF Shooting", "Heading Delta X", headingDeltaX);
		LightningShuffleboard.setDouble("OTF Shooting", "Heading Delta Y", headingDeltaY);
		LightningShuffleboard.setDouble("OTF Shooting", "Robot X Velocity", robotVelocityX);
		LightningShuffleboard.setDouble("OTF Shooting", "Robot Y Velocity", robotVelocityY);
		LightningShuffleboard.setDouble("OTF Shooting", "Robot X Accleration", robotAccelerationX);
		LightningShuffleboard.setDouble("OTF Shooting", "Robot Y Accleration", robotAccelerationY);
		LightningShuffleboard.setDouble("OTF Shooting", "Time to Speaker", timeToSpeaker);
		LightningShuffleboard.setDouble("OTF Shooting", "Release Robot X", releaseRobotPoseX);
		LightningShuffleboard.setDouble("OTF Shooting", "Release Robot Y", releaseRobotPoseY);
		LightningShuffleboard.setDouble("OTF Shooting", "Target X", targetX);
		LightningShuffleboard.setDouble("OTF Shooting", "Target Y", targetY);
		LightningShuffleboard.setDouble("OTF Shooting", "Impact Location Delta X", pieceDeltaX);
		LightningShuffleboard.setDouble("OTF Shooting", "Impact Location Delta Y", pieceDeltaY);
		LightningShuffleboard.setDouble("OTF Shooting", "PID Output", pidOutput);
		LightningShuffleboard.setDouble("OTF Shooting", "Robot X", pose.getX());
		LightningShuffleboard.setDouble("OTF Shooting", "Robot Y", pose.getY());
		LightningShuffleboard.setDouble("OTF Shooting", "Robot Heading", pose.getRotation().getDegrees());
		LightningShuffleboard.setDouble("OTF Shooting", "Basic Delta", basicDelta);
		LightningShuffleboard.setDouble("OTF Shooting", "Rotated Basic Heading", (basicHeading + 360) % 360);
		LightningShuffleboard.setDouble("OTF Shooting", "Basic Heading", basicHeading);
		LightningShuffleboard.setDouble("OTF Shooting", "Driver X", driver.getLeftX() * DrivetrainConstants.MaxSpeed);
		LightningShuffleboard.setDouble("OTF Shooting", "Driver Y", driver.getLeftY() * DrivetrainConstants.MaxSpeed);
		LightningShuffleboard.setDouble("OTF Shooting", "Driver Rot", driver.getRightX() * DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT);
		LightningShuffleboard.setDouble("OTF Shooting", "Velocity Scaler", veloctiyScaler);
		LightningShuffleboard.setDouble("OTF Shooting", "Acceleration Scaler", accelerationScaler);

		// Normal driving, no OTF or point at tags
		// drivetrain.applyRequestField(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX());


		// OTF driving
		drivetrain.setFieldDriver(-driver.getLeftY(), -driver.getLeftX(), pidOutput);

		pivot.setTargetAngle(pivotTargetAngle);
		flywheel.setAllMotorsRPM(0); //TODO: get real value

		if (fireTime - Timer.getFPGATimestamp() < 0) {
			indexer.setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
		} 
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}