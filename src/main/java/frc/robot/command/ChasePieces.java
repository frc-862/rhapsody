package frc.robot.command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShuffleboardPeriodicConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;
import frc.thunder.vision.Limelight;

public class ChasePieces extends Command {

	private Swerve drivetrain;
	private Collector collector;
	private Indexer indexer;
	private Pivot pivot;
	private Limelight limelight;

	private double pidOutput;
	private double targetHeading;
	private double previousTargetHeading;

	private double targetPitch;
	private double power;

    private boolean onTarget;
	private boolean hasPiece;
	private boolean isDone;
	private boolean hasTarget;

	private Command smartCollect;
	private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

	private Debouncer debouncer = new Debouncer(IndexerConstants.INDEXER_DEBOUNCE_TIME);

	private LightningShuffleboardPeriodic periodicShuffleboard;

	/**
	 * Creates a new ChasePieces.
	 * @param drivetrain to request movement
	 * @param collector for smart collect
	 * @param indexer for smart collect
	 * @param pivot for smart collect
	 * @param limelights to get vision data from dust
	 */

	public ChasePieces(Swerve drivetrain, Collector collector, Indexer indexer, Pivot pivot, Limelights limelights) {
		this.drivetrain = drivetrain;
		this.collector = collector;
		this.indexer = indexer;
		this.pivot = pivot;

		limelight = limelights.getDust();

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
		power = 0d;
		smartCollect =  new SmartCollect(() -> power, () -> power, collector, indexer, pivot);
		initLogging();
		smartCollect.schedule();
	}

	@SuppressWarnings("unchecked")
	private void initLogging() {
		periodicShuffleboard = new LightningShuffleboardPeriodic("ChasePieces", ShuffleboardPeriodicConstants.DEFAULT_SHUFFLEBOARD_PERIOD,
				new Pair<String, Object>("On Target", (BooleanSupplier) () -> onTarget),
				new Pair<String, Object>("Has Target", (BooleanSupplier) () -> hasTarget),
				new Pair<String, Object>("Is Done", (BooleanSupplier) () -> isDone),
				new Pair<String, Object>("Has Piece", (BooleanSupplier) () -> hasPiece),
				new Pair<String, Object>("Target Heading", (DoubleSupplier) () -> targetHeading),
				new Pair<String, Object>("Target Y", (DoubleSupplier) () -> targetPitch),
				new Pair<String, Object>("Pid Output", (DoubleSupplier) () -> pidOutput),
				new Pair<String, Object>("SmartCollectPower", (DoubleSupplier) () -> power));
	}

	@Override
	public void execute() {
		hasTarget = limelight.hasTarget();

		if (hasTarget){
			previousTargetHeading = targetHeading;
			targetHeading = limelight.getTargetX();
			targetPitch = limelight.getTargetY();
		}

		onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
		isDone = smartCollect.isFinished();
		hasPiece = debouncer.calculate(indexer.getEntryBeamBreakState()) || collector.getEntryBeamBreakState();

		pidOutput = headingController.calculate(0, targetHeading);

		if (!hasPiece){
			if (hasTarget){
				if (trustValues()){
					power = 0.65d;
					if (!onTarget) {
						drivetrain.setRobot(3, 0, -pidOutput);
					} else {
						drivetrain.setRobot(3, 0, 0);
					}
				}
			} else {
				drivetrain.setRobot(3, 0, 0);
			}
		} else {
			drivetrain.setRobot(0, 0, 0);
		}

	}

	@Override
	public void end(boolean interrupted) {
		power = 0d;
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
		return isDone;
	}
}