package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class ComboPoint extends Command {
    private double minPower = 0.3;

    private Swerve drivetrain;
    private XboxControllerFilter driver;
    private Limelight stopMe;

    private double pidOutput;
    private double feedForwardOutput;
    private double targetHeading;
    private double currentHeading;
    private double targetBias = 0d;
    private Translation2d targetPose;
    private Translation2d originalTargetPose;

    private PIDController pidController = VisionConstants.COMBO_CONTROLLER; // TAG_AIM_CONTROLLER
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25, 0.5);

    private Debouncer debouncer = new Debouncer(0.2);

    private DoubleLogEntry deltaYLog;
    private DoubleLogEntry deltaXLog;
    private DoubleLogEntry targetHeadingLog;
    private DoubleLogEntry targetYLog;
    private DoubleLogEntry targetXLog;
    private DoubleLogEntry pidOutputLog;
    private DoubleLogEntry targetMinusCurrentHeadingLog;
    private DoubleLogEntry currentHeadingLog;
    private BooleanLogEntry inToleranceLog;

    /**
     * Creates a new ComboPoint.
     *
     * @param targetPose the target pose to point at
     * @param drivetrain to request movement
     * @param driver the driver's controller, used for drive input
     * @param limelights for tag align
     * @param bias the bias to add from the target heading
     */
    public ComboPoint(Translation2d targetPose, Swerve drivetrain, XboxControllerFilter driver, Limelights limelights, double bias) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.originalTargetPose = targetPose;
        this.stopMe = limelights.getStopMe();
        this.targetBias = bias;

        addRequirements(drivetrain);

        initLogging();
    }

    @Override
    public void initialize() {
        pidController.enableContinuousInput(0, 360);
        pidController.setTolerance(VisionConstants.POINTATTAG_ALIGNMENT_TOLERANCE);

        if (isBlueAlliance()) {
            targetPose = originalTargetPose;
            stopMe.setPriorityTag(7);
        } else {
            targetPose = swapAlliance(originalTargetPose);
            stopMe.setPriorityTag(4);
        }

        stopMe.setFiducialIDFiltersOverride(VisionConstants.SPEAKER_FILTERS);// Hopefully the above change makes this unnecessary

        System.out.println("DRIVE - COMBO POINT START");
    }

    private boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /**
     * Used to swap the target pose from blue alliance to red alliance
     * 
     * @param pose the pose to swap
     * @return the new pose
     */
    private Translation2d swapAlliance(Translation2d pose) {
        return new Translation2d(PoseConstants.FIELD_LIMIT.getX() - pose.getX(), pose.getY());
    }

    private boolean inTolerance() {
        double difference = Math.abs(currentHeading - targetHeading);
        difference = difference > 180 ? 360 - difference : difference;
        return difference <= VisionConstants.POINTATTAG_ALIGNMENT_TOLERANCE && stopMe.hasTarget(); // TODO not has Target but if the correct filter is set
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose();
        var deltaX = targetPose.getX() - pose.getX();
        var deltaY = targetPose.getY() - pose.getY();

        currentHeading = (pose.getRotation().getDegrees() + 360) % 360;

        if (stopMe.hasTarget()) {
            targetHeading = (stopMe.getTargetX() - 1.592) + targetBias;
            targetHeading = currentHeading - targetHeading;
        } else {
            // Calculate vector to target, add 180 to make it point backwards
            targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180; 
        }

        targetHeading = (targetHeading + 360) % 360; // Modulo 360 to keep it in the range of 0-360
        
        // if (!inTolerance() && Math.abs(pidOutput) < minPower) {
        // pidOutput = Math.signum(pidOutput) * minPower;
        // }

        pidOutput = pidController.calculate(currentHeading, targetHeading);
        feedForwardOutput = feedforward.calculate(pidOutput);
        
        if(inTolerance()) {
            feedForwardOutput = 0;
        }
        drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), feedForwardOutput);

        updateLogging();

        tuning();
    }

    public void tuning() {
        pidController.setP(LightningShuffleboard.getDouble("ComboPoint", "P", pidController.getP()));
        pidController.setI(LightningShuffleboard.getDouble("ComboPoint", "I", pidController.getI()));
        pidController.setD(LightningShuffleboard.getDouble("ComboPoint", "D", pidController.getD()));
        minPower = LightningShuffleboard.getDouble("ComboPoint", "Min Power", minPower);
    }

    /**
     * Set log paths
     */
    public void initLogging() {
        DataLog log = DataLogManager.getLog();

        deltaYLog = new DoubleLogEntry(log, "/ComboPoint/Delta Y");
        deltaXLog = new DoubleLogEntry(log, "/ComboPoint/Delta X");
        targetHeadingLog = new DoubleLogEntry(log, "/ComboPoint/Target Heading");
        targetYLog = new DoubleLogEntry(log, "/ComboPoint/Target Pose Y");
        targetXLog = new DoubleLogEntry(log, "/ComboPoint/Target Pose X");
        pidOutputLog = new DoubleLogEntry(log, "/ComboPoint/Pid Output");
        targetMinusCurrentHeadingLog = new DoubleLogEntry(log, "/ComboPoint/Target minus current heading");
        currentHeadingLog = new DoubleLogEntry(log, "/ComboPoint/Current-Heading");
        inToleranceLog = new BooleanLogEntry(log, "/ComboPoint/InTolerance");
    }

    /**
     * update logging with values
     */
    public void updateLogging() {
        deltaYLog.append(targetPose.getY() - drivetrain.getPose().getY());
        deltaXLog.append(targetPose.getX() - drivetrain.getPose().getX());
        targetHeadingLog.append(targetHeading);
        targetYLog.append(targetPose.getY());
        targetXLog.append(targetPose.getX());
        pidOutputLog.append(pidOutput);
        targetMinusCurrentHeadingLog.append(targetHeading - currentHeading);
        currentHeadingLog.append(currentHeading);
        inToleranceLog.append(inTolerance());

        if (!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDouble("ComboPoint", "Target Heading", targetHeading);
            LightningShuffleboard.setDouble("ComboPoint", "Current Heading", currentHeading);
            LightningShuffleboard.setBool("ComboPoint", "In Tolerance", inTolerance());
            LightningShuffleboard.setDouble("ComboPoint", "Raw Output (PointController)", pidOutput);
            LightningShuffleboard.setDouble("ComboPoint", "Output (FeedForward)", feedForwardOutput);
            LightningShuffleboard.setBool("ComboPoint", "HasTarget", stopMe.hasTarget());
            LightningShuffleboard.setDouble("ComboPoint", "Bias", targetBias);
        }
    }

    @Override
    public void end(boolean interrupted) {
        stopMe.setFiducialIDFiltersOverride(VisionConstants.ALL_TAG_FILTERS);
        System.out.println("DRIVE - COMBO POINT END");
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous()) {
            return debouncer.calculate(inTolerance());
        }
        return false;
    }
}
