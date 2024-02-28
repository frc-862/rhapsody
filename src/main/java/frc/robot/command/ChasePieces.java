package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
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

		addRequirements(drivetrain, collector);
	}

	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		collector.setPower(1d); //TODO: get the right power

		initLogging();
	}

	private void initLogging() {
		LightningShuffleboard.setBoolSupplier("ChasePieces", "On Target", () -> onTarget);
		LightningShuffleboard.setBoolSupplier("ChasePieces", "Has Target", () -> hasTarget);

		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Target Heading", () -> targetHeading);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Target Y", () -> targetPitch);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Pid Output", () -> pidOutput);
	}

	@Override
	public void execute() {
		// For tuning.
		// headingController.setP(LightningShuffleboard.getDouble("ChasePieces", "Pid P", 0.05));
		// headingController.setI(LightningShuffleboard.getDouble("ChasePieces", "Pid I", 0));
		// headingController.setD(LightningShuffleboard.getDouble("ChasePieces", "Pid D", 0));

		hasTarget = limelight.hasTarget();

		if (hasTarget){
			previousTargetHeading = targetHeading;
			targetHeading = limelight.getTargetX();
			targetPitch = limelight.getTargetY();
		}

		onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
		hasPiece = collector.hasPiece();

		pidOutput = headingController.calculate(0, targetHeading);

		if (!hasPiece){
			if (hasTarget){
				if (trustValues()){
					if (!onTarget) {
						drivetrain.setRobot(3, 0, -pidOutput);
					} else {
						drivetrain.setRobot(3, 0, 0);
					}
				}
			} else {
				drivetrain.setRobot(3, 0, 0);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
	}

	/**
	 * Makes sure that the robot isn't jerking over to a different side while chasing pieces.
	 * @return t/f if the robot should trust the values
	 */
	public boolean trustValues(){
		if ((Math.abs(targetHeading) - Math.abs(previousTargetHeading)) < 6){
			return true;
		}
		return false;
	}

	@Override
	public boolean isFinished() {
		if (hasPiece){
			return true;
		}
		return false;
	}
}