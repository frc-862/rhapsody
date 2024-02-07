package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class ChasePieces extends Command {

	private Swerve drivetrain;
	private Collector collector;
	private Limelight limelight;

	private RobotCentric noteChase;
	
	private int limelightId = 0;

	private double pidOutput;
	private double targetHeading;
	private double targetPitch;

    private boolean onTarget;
	private boolean hasPiece;

	private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

	/**
	 * Creates a new ChasePieces.
	 * @param drivetrain to request movement 
	 * @param collector to collect pieces
	 * @param limelights to get vision data from dust
	 */
	public ChasePieces(Swerve drivetrain, Collector collector, Limelights limelights) {
		this.drivetrain = drivetrain;
		this.collector = collector;

		limelight = limelights.getDust();
		limelightId = limelight.getPipeline();
		
		noteChase = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

		addRequirements(drivetrain, collector);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		limelight.setPipeline(VisionConstants.NOTE_PIPELINE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		targetHeading = limelight.getTargetX();
		targetPitch = limelight.getTargetY();

		onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
		hasPiece = collector.getEntryBeamBreakState();

		LightningShuffleboard.setBool("ChasePieces", "On Target", onTarget);

		LightningShuffleboard.setDouble("ChasePieces", "Drivetrain Angle", drivetrain.getPigeon2().getAngle());
		LightningShuffleboard.setDouble("ChasePieces", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("ChasePieces", "Target Y", targetPitch);
		LightningShuffleboard.setDouble("ChasePieces", "Pid Output", pidOutput);

		// For tuning.
		// headingController.setP(LightningShuffleboard.getDouble("ChasePieces", "Pid P", 0.05));
		// headingController.setI(LightningShuffleboard.getDouble("ChasePieces", "Pid I", 0));
		// headingController.setD(LightningShuffleboard.getDouble("ChasePieces", "Pid D", 0));


		pidOutput = headingController.calculate(0, targetHeading);
		
		if (!hasPiece){
			if (!onTarget) {
				drivetrain.setControl(
					noteChase.withRotationalRate(-pidOutput).withVelocityX(3) // Should be positive for front of robot, negative for back of robot.
				);
			} else {
				drivetrain.setControl(
					noteChase.withVelocityX(3) // Should be positive for front of robot, negative for back of robot.
				);
			}

			// Activates if the target is close to the drivetrain.
			if (targetPitch < 0) {
				collector.setPower(0.2); // TODO: get the proper value to set to the collector.
			}
		} else {
			end(false);
		}
        
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightId);
		collector.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}