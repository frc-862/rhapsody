package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Limelights;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class ChasePieces extends Command {

	private Swerve drivetrain;
	private Collector collector;
	private Indexer indexer;
	private Flywheel flywheel;
	private Pivot pivot;
	private Limelight limelight;

	private double pidOutput;
	private double targetHeading;
	private double previousTargetHeading;

	private double targetPitch;
	private double collectPower;
	private double maxCollectPower;
	private double drivePower;

    private boolean onTarget;
	private boolean hasPiece;
	private boolean isDone;
	private boolean hasTarget;

	private Command smartCollect;
	private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

	private Debouncer debouncer = new Debouncer(IndexerConstants.INDEXER_DEBOUNCE_TIME);

	/**
	 * Creates a new ChasePieces.
	 * @param drivetrain to request movement
	 * @param collector for smart collect
	 * @param indexer for smart collect
	 * @param flywheel for stopping the flywheels before collecting / for smart collect
	 * @param pivot for smart collect
	 * @param limelights to get vision data from dust
	 */
	public ChasePieces(Swerve drivetrain, Collector collector, Indexer indexer, Pivot pivot, Flywheel flywheel, Limelights limelights) {
		this.drivetrain = drivetrain;
		this.collector = collector;
		this.indexer = indexer;
		this.flywheel = flywheel;
		this.pivot = pivot;

		this.limelight = limelights.getDust();

		addRequirements(drivetrain, collector, indexer, flywheel);
	}

	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		collectPower = 0d;
		smartCollect = new SmartCollect(() -> collectPower, () -> collectPower, collector, indexer, pivot, flywheel);

		if (DriverStation.isAutonomous()){
			drivePower = 1.5d;
			maxCollectPower = 0.5d;
		} else {
			maxCollectPower = 0.65d;
			drivePower = 3d;
		}

		initLogging();
		smartCollect.initialize();
	}

	private void initLogging() {
		LightningShuffleboard.setBoolSupplier("ChasePieces", "On Target", () -> onTarget);
		LightningShuffleboard.setBoolSupplier("ChasePieces", "Has Target", () -> hasTarget);
		LightningShuffleboard.setBoolSupplier("ChasePieces", "Is Done", () -> isDone);
		LightningShuffleboard.setBoolSupplier("ChasePieces", "Has Piece", () -> hasPiece);

		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Target Heading", () -> targetHeading);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Target Y", () -> targetPitch);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "Pid Output", () -> pidOutput);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "SmartCollectPower", () -> collectPower);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "DrivePower", () -> drivePower);
		LightningShuffleboard.setDoubleSupplier("ChasePieces", "MaxCollectPower", () -> maxCollectPower);


	}

	@Override
	public void execute() {
		hasTarget = limelight.hasTarget();
		smartCollect.execute();

		if (hasTarget){
			previousTargetHeading = targetHeading;
			targetHeading = limelight.getTargetX();
			targetPitch = limelight.getTargetY();
		}

		onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
		hasPiece = debouncer.calculate(indexer.getEntryBeamBreakState()) || collector.getEntryBeamBreakState();

		pidOutput = headingController.calculate(0, targetHeading);

		if (!hasPiece){
			if (hasTarget){
				if (trustValues()){
					collectPower = maxCollectPower;
					if (!onTarget) {
						drivetrain.setRobot(drivePower, 0, -pidOutput);
					} else {
						drivetrain.setRobot(drivePower, 0, 0);
					}
				}
			} else {
				drivetrain.setRobot(drivePower, 0, 0);
			}
		} else {
			collectPower = 0d;
			drivetrain.setRobot(0, 0, 0);
		}

	}

	@Override
	public void end(boolean interrupted) {
		collectPower = 0d;
		smartCollect.end(interrupted);
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
		return smartCollect.isFinished();
	}
}