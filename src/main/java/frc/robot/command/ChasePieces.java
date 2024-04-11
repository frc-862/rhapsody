package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
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

    private final double collectPower = 1d;
    private double defaultDrivePower;
    private double drivePower;
    private double rotPower;
    private final double defaultRotPower = 1.5d;
    private boolean hasTarget;
    private boolean hasSeenTarget;

    private Command smartCollect;
    private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

    private BooleanLogEntry onTargetLog;
    private BooleanLogEntry hasTargetLog;
    private BooleanLogEntry trustValuesLog;

    private DoubleLogEntry targetHeadingLog;
    private DoubleLogEntry pidOutputLog;
    private DoubleLogEntry drivePowerLog;
    private DoubleLogEntry rotPowerLog;

    /**
     * Creates a new ChasePieces.
     *
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

        if (DriverStation.isAutonomous()) {
            this.defaultDrivePower = 1.5d;
        } else {
            this.defaultDrivePower = 5d;
        }

        addRequirements(drivetrain, collector, indexer, flywheel);

        initLogging();
    }

    @Override
    public void initialize() {
        System.out.println("AUTO - Chase pieces INIT");

        headingController.setTolerance(VisionConstants.CHASE_PIECE_ALIGNMENT_TOLERANCE);
        headingController.setSetpoint(0);

        if (DriverStation.isAutonomous()) {
            smartCollect = new AutonSmartCollect(() -> collectPower, () -> collectPower, collector, indexer);
        } else {
            smartCollect = new SmartCollect(() -> collectPower, () -> collectPower, collector, indexer, pivot, flywheel);
        }

        smartCollect.initialize();

        hasSeenTarget = false;
    }

    /**
     * Initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        onTargetLog = new BooleanLogEntry(log, "/ChasePieces/On Target");
        hasTargetLog = new BooleanLogEntry(log, "/ChasePieces/Has Target");
        trustValuesLog = new BooleanLogEntry(log, "/ChasePieces/Trust Values");

        targetHeadingLog = new DoubleLogEntry(log, "/ChasePieces/Target Heading");
        pidOutputLog = new DoubleLogEntry(log, "/ChasePieces/Pid Output");
        drivePowerLog = new DoubleLogEntry(log, "/ChasePieces/DrivePower");
        rotPowerLog = new DoubleLogEntry(log, "/ChasePieces/RotPower");
    }

    @Override
    public void execute() {
        hasTarget = limelight.hasTarget();
        smartCollect.execute();

        if (hasTarget) { // If limelight sees a note, set heading offset
            previousTargetHeading = targetHeading;
            targetHeading = limelight.getTargetX();
        }

        pidOutput = headingController.calculate(targetHeading);

        if (headingController.atSetpoint()) {
            pidOutput = 0;
        }



        if (DriverStation.isAutonomousEnabled()) {
            checkSlowdown();

            if (hasTarget) { // If limelight sees a note
                drivePower = defaultDrivePower;
                if (trustValues()) {
                    hasSeenTarget = true;
                    rotPower = pidOutput;
                }
            } else {
                if (!hasSeenTarget) { // If the limelight never saw a note
                    drivePower = 0.5;
                    if (drivetrain.getPose().getY() > VisionConstants.HALF_FIELD_HEIGHT) {
                        rotPower = DriverStation.getAlliance().get() == Alliance.Blue ? -defaultRotPower : defaultRotPower;
                    } else {
                        rotPower = DriverStation.getAlliance().get() == Alliance.Blue ? defaultRotPower : -defaultRotPower;
                    }
                } else { // IF seen target, but lost it (Under bumper most likely)
                    drivePower = defaultDrivePower;
                    rotPower = 0;
                }
            }
        }

        else { // If in teleop
            drivePower = defaultDrivePower;
            if (hasTarget) { // If limelight sees a note
                if (trustValues()) {
                    rotPower = pidOutput;
                }
            } else { // If not drive forward
                rotPower = 0;
            }
        }

        drivetrain.setRobot(drivePower, 0, rotPower);

        updateLogging();
    }

    /**
     * Update logging
     */
    private void updateLogging() {
        onTargetLog.append(headingController.atSetpoint());
        hasTargetLog.append(hasTarget);
        trustValuesLog.append(trustValues());

        targetHeadingLog.append(targetHeading);
        pidOutputLog.append(pidOutput);
        drivePowerLog.append(drivePower);
        rotPowerLog.append(rotPower);


        if (!DriverStation.isAutonomous()) {
            LightningShuffleboard.setBool("ChasePieces", "Is Running", isFinished());
            LightningShuffleboard.setDouble("ChasePieces", "DrivePower", defaultDrivePower);
            LightningShuffleboard.setDouble("ChasePieces", "Current X", drivetrain.getPose().getX());
        }
    }

    private void checkSlowdown() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (drivetrain.getPose().getX() > AutonomousConstants.BLUE_SLOW_CHASE_RANGE) {
                defaultDrivePower = 0.5d;
            } else {
                defaultDrivePower = 1.5d;
            }
        } else {
            if (drivetrain.getPose().getX() < AutonomousConstants.RED_SLOW_CHASE_RANGE) {
                defaultDrivePower = 0.5d;
            } else {
                defaultDrivePower = 1.5d;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        smartCollect.end(interrupted);
        drivetrain.setRobot(0, 0, 0);
        System.out.println("AUTO - Chase pieces END");
    }

    /**
     * Makes sure that the robot isn't jerking over to a different side while chasing pieces.
     * 
     * @return t/f if the robot should trust the values
     */
    public boolean trustValues() {
        return (Math.abs(targetHeading) - Math.abs(previousTargetHeading)) < 6d;
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                if (drivetrain.getPose().getX() > AutonomousConstants.BLUE_CHASE_BOUNDARY) {
                    return true;
                }
            } else {
                if (drivetrain.getPose().getX() < AutonomousConstants.RED_CHASE_BOUNDARY) {
                    return true;
                }
            }
            return indexer.hasNote(); // If any beam break sees a note, stop chasing
        } else {
            // return smartCollect.isFinished();
            return indexer.getPieceState() == PieceState.IN_COLLECT;
        }
    }
}
