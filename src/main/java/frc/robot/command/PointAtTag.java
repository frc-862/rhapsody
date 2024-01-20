package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class PointAtTag extends Command {

	private Swerve drivetrain;
	private Limelight[] limelights;
	private double targetHeading;
	private double lockedOnHeading;
	private double pidOutput;
	private SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	private boolean useLimelights;

	PIDController headingController = VisionConstants.HEADING_CONTROLLER;

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 * @param limelights to get vision data
	 */
	public PointAtTag(Swerve drivetrain, boolean useLimelights) {
		this.drivetrain = drivetrain;
		this.useLimelights = useLimelights;
		limelights = drivetrain.getLimelights();

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (!useLimelights) {
			lockedOnHeading = LightningShuffleboard.getDouble("PointAtTag", "LockOnHeading", 0);
			LightningShuffleboard.setDouble("PointAtTag", "Drivetrain Angle", drivetrain.getPigeon2().getAngle());
			targetHeading = lockedOnHeading - drivetrain.getPigeon2().getAngle();


			SwerveRequest.FieldCentric pointAtTag = new SwerveRequest.FieldCentric();
			pidOutput = headingController.calculate(0, targetHeading);
			drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));

		} else {
			for (Limelight limelight : Limelight.filterLimelights(limelights)) {
				targetHeading = limelight.getTargetX();
			}

			SwerveRequest.RobotCentric pointAtTag = new SwerveRequest.RobotCentric();
			pidOutput = headingController.calculate(0, targetHeading);
			drivetrain.setControl(pointAtTag.withRotationalRate(-pidOutput));

		}

		LightningShuffleboard.setDouble("PointAtTag", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("PointAtTag", "Pid Output", pidOutput);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.applyRequest(() -> brake); // TODO test if this applies brake
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
