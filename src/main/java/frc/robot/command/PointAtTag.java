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
	private double pidOutput;
	private SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	PIDController headingController = new PIDController(0.1, 0, 0); // TODO make constant and tune

	/**
	 * Creates a new PointAtTag.
	 * @param drivetrain to request movement 
	 */
	public PointAtTag(Swerve drivetrain) {
		this.drivetrain = drivetrain;
		this.limelights = drivetrain.getLimelights();

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
