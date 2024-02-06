package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class ChasePieces extends Command {

	private Swerve drivetrain;
	private Limelight limelight;

	private RobotCentric noteChase;
	
	private int limelightId = 0;
	private double pidOutput;
	private double targetHeading;
	private double previousTargetHeading;


    private boolean onTarget;
	private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

	/**
	 * Creates a new ChasePieces.
	 * @param drivetrain to request movement 
	 * @param limelights to get the limelight from
	 */
	public ChasePieces(Swerve drivetrain, Limelights limelights) {
		this.drivetrain = drivetrain;

		limelight = limelights.getDust();
		limelightId = limelight.getPipeline();

		limelight.setPipeline(VisionConstants.NOTE_PIPELINE);
		
		noteChase = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		previousTargetHeading = targetHeading;
		targetHeading = limelight.getTargetX();

        if(Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE){
            onTarget = true;
        } else{
            onTarget = false;
        }

		LightningShuffleboard.setBool("ChasePieces", "On Target", onTarget);
		LightningShuffleboard.setDouble("ChasePieces", "Drivetrain Angle", drivetrain.getPigeon2().getAngle());
		LightningShuffleboard.setDouble("ChasePieces", "Target Heading", targetHeading);
		LightningShuffleboard.setDouble("ChasePieces", "Pid Output", pidOutput);

		headingController.setP(LightningShuffleboard.getDouble("ChasePieces", "Pid P", 0.05));
		headingController.setI(LightningShuffleboard.getDouble("ChasePieces", "Pid I", 0));
		headingController.setD(LightningShuffleboard.getDouble("ChasePieces", "Pid D", 0));


		pidOutput = headingController.calculate(0, targetHeading);
		
		if (trustValues()){
			drivetrain.setControl(noteChase.withRotationalRate(-pidOutput) 
				.withVelocityX(-3) // Should be positive for front of robot, negative for back of robot.
			);
		}
        
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setPipeline(limelightId);
	}

	// Makes sure that the robot isn't jerking over to a different side while chasing pieces.
	public boolean trustValues(){
		if ((targetHeading - previousTargetHeading) > 5){ //TODO: find this magic number and make it a constant
			return false;
		} else {
			return true;
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}