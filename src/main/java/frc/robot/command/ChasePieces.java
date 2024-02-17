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
	
	private int limelightId = 0;

	private double pidOutput;
	private double targetHeading;
	private double previousTargetHeading;

	private double targetPitch;

    private boolean onTarget;
	private boolean hasPiece;
	private boolean hasTarget;

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
		

		addRequirements(drivetrain, collector);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		limelight.setPipeline(VisionConstants.NOTE_PIPELINE);
		collector.setPower(1d); //TODO: get the right power
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		hasTarget = limelight.hasTarget();

		if (hasTarget){
			previousTargetHeading = targetHeading;
			targetHeading = limelight.getTargetX();
			targetPitch = limelight.getTargetY();
		}

		onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
		hasPiece = collector.hasPiece();

		LightningShuffleboard.setBool("ChasePieces", "On Target", onTarget);
		LightningShuffleboard.setBool("ChasePieces", "Has Target", hasTarget);

		LightningShuffleboard.setDouble("ChasePieces", "Drivetrain Angle", drivetrain.getPigeon2().getAngle());
		LightningShuffleboard.setDouble("ChasePieces", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("ChasePieces", "Target Y", targetPitch);
		LightningShuffleboard.setDouble("ChasePieces", "Pid Output", pidOutput);

		// For tuning.
		// headingController.setP(LightningShuffleboard.getDouble("ChasePieces", "Pid P", 0.05));
		// headingController.setI(LightningShuffleboard.getDouble("ChasePieces", "Pid I", 0));
		// headingController.setD(LightningShuffleboard.getDouble("ChasePieces", "Pid D", 0));


		pidOutput = headingController.calculate(0, targetHeading);
		
		if (hasTarget){
			if (trustValues()){
				if (!onTarget) {
					drivetrain.setRobot(-3, 0, -pidOutput);
				} else {
					drivetrain.setRobot(-3, 0, 0);
				}

			}
		} else {
			drivetrain.setRobot(3, 0, 0);
		}

		isFinished();
        
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightId);
		collector.stop();
	}

	// Makes sure that the robot isn't jerking over to a different side while chasing pieces.
	public boolean trustValues(){
		if ((Math.abs(targetHeading) - Math.abs(previousTargetHeading)) < 6){
			return true;
		}
		return false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (hasPiece){
			return true;
		}
		
		return false;
	}
}